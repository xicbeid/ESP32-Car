/*
 * Simplified face recognition - NCC template matching
 * Zero model dependencies, uses MSRMNP detection + keypoints
 */
extern "C" {
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "face_recog.h"
#include "human_detect.h"
}

#define TAG "FaceRec"
#define TMPL_W 112
#define TMPL_H 112
#define TMPL_BYTES (TMPL_W*TMPL_H)
#define REC_PER_FACE (2+32+TMPL_BYTES)

static struct {
    bool initialized, enabled;
    char db_path[64];
    fr_recog_result_t latest;
    SemaphoreHandle_t mutex;
} g_fr;

static esp_err_t spiffs_mount(const char *l, const char *m) {
    esp_vfs_spiffs_conf_t c = {.base_path=m, .partition_label=l, .max_files=5, .format_if_mount_failed=true};
    return esp_vfs_spiffs_register(&c);
}

static void extract_aligned_face(const uint8_t *rgb, int w, int h, int stride,
                                  const hd_box_t *box, uint8_t *out) {
    int cx=box->x, cy=box->y, cw=box->w, ch=box->h;
    if(cx<0){cw+=cx;cx=0;} if(cy<0){ch+=cy;cy=0;}
    if(cx+cw>w)cw=w-cx; if(cy+ch>h)ch=h-cy;
    if(cw<10||ch<10){memset(out,0x80,TMPL_BYTES);return;}
    int px=cw/10, py=ch/10;
    cx-=px; cw+=px*2; cy-=py; ch+=py*2;
    if(cx<0){cw+=cx;cx=0;} if(cy<0){ch+=cy;cy=0;}
    if(cx+cw>w)cw=w-cx; if(cy+ch>h)ch=h-cy;
    for(int y=0;y<TMPL_H;y++){
        float sy=(float)y/TMPL_H*ch+cy; int isy=(int)sy; float fy=sy-isy;
        if(isy<0)isy=0; if(isy>=h-1)isy=h-2;
        for(int x=0;x<TMPL_W;x++){
            float sx=(float)x/TMPL_W*cw+cx; int isx=(int)sx; float fx=sx-isx;
            if(isx<0)isx=0; if(isx>=w-1)isx=w-2;
            const uint16_t *p00=(const uint16_t*)(rgb+isy*stride+isx*2);
            const uint16_t *p10=(const uint16_t*)(rgb+isy*stride+(isx+1)*2);
            const uint16_t *p01=(const uint16_t*)(rgb+(isy+1)*stride+isx*2);
            const uint16_t *p11=(const uint16_t*)(rgb+(isy+1)*stride+(isx+1)*2);
            float g00=(float)((*p00>>5)&0x3F), g10=(float)((*p10>>5)&0x3F);
            float g01=(float)((*p01>>5)&0x3F), g11=(float)((*p11>>5)&0x3F);
            float g=(g00*(1-fx)+g10*fx)*(1-fy)+(g01*(1-fx)+g11*fx)*fy;
            out[y*TMPL_W+x]=(uint8_t)(g*4.0f+0.5f);
        }
    }
}

static float ncc_match(const uint8_t *a, const uint8_t *b, int n) {
    float sa=0,sb=0; int nn=0;
    for(int i=0;i<n;i++){if(a[i]<16&&b[i]<16)continue;sa+=a[i];sb+=b[i];nn++;}
    if(nn<100)return 0;
    float ma=sa/nn, mb=sb/nn, num=0, da2=0, db2=0;
    for(int i=0;i<n;i++){if(a[i]<16&&b[i]<16)continue;
        float da=a[i]-ma, db=b[i]-mb; num+=da*db; da2+=da*da; db2+=db*db;}
    float den=sqrtf(da2*db2);
    return den>1e-6f?num/den:0;
}

static void db_match(const uint8_t *q, fr_recog_result_t *out) {
    memset(out,0,sizeof(*out));
    FILE *f=fopen(g_fr.db_path,"rb"); if(!f)return;
    uint16_t cnt; if(fread(&cnt,sizeof(cnt),1,f)!=1){fclose(f);return;}
    uint8_t *buf=(uint8_t*)malloc(REC_PER_FACE); if(!buf){fclose(f);return;}
    for(int i=0;i<cnt&&out->count<FR_MAX_FACES;i++){
        if(fread(buf,1,REC_PER_FACE,f)<REC_PER_FACE)break;
        uint16_t id=*(uint16_t*)buf; if(id==0)continue;
        float s=ncc_match(q,buf+2+32,TMPL_BYTES);
        if(s>0.50f){out->matches[out->count].id=id;out->matches[out->count].similarity=s;out->count++;}
    }
    free(buf); fclose(f); out->recognized=(out->count>0);
}

extern "C" {

esp_err_t face_recognition_init(const char *pl, const char *mp, const char *db) {
    if(g_fr.initialized)return ESP_OK;
    memset(&g_fr,0,sizeof(g_fr));
    strncpy(g_fr.db_path,db,sizeof(g_fr.db_path)-1);
    g_fr.enabled=false;
    esp_err_t e=spiffs_mount(pl,mp);
    if(e!=ESP_OK){e=spiffs_mount(pl,mp);if(e!=ESP_OK)return e;}
    FILE *f=fopen(db,"rb");
    if(!f){f=fopen(db,"wb");if(f){uint16_t z=0;fwrite(&z,sizeof(z),1,f);fclose(f);}}
    else{uint16_t c;fread(&c,sizeof(c),1,f);fclose(f);ESP_LOGI(TAG,"DB: %d faces",(int)c);}
    g_fr.mutex=xSemaphoreCreateMutex();
    g_fr.initialized=true;
    ESP_LOGI(TAG,"Face recog ready (NCC, zero-model)");
    return ESP_OK;
}

void face_recognition_set_enabled(bool en){g_fr.enabled=en;}
bool face_recognition_is_enabled(void){return g_fr.enabled;}

esp_err_t face_recognition_enroll(const uint8_t *rgb, int w, int h, int stride,
                                   const char *name, const int box[4], uint16_t *oid) {
    *oid=0;
    if(!g_fr.initialized||!rgb||!name)return ESP_ERR_INVALID_ARG;
    hd_result_t det=human_detect_get_results();
    if(det.count==0)return ESP_FAIL;
    hd_box_t fb; fb.x=box[0];fb.y=box[1];fb.w=box[2];fb.h=box[3];
    uint8_t tmpl[TMPL_BYTES];
    extract_aligned_face(rgb,w,h,stride,&fb,tmpl);
    FILE *f=fopen(g_fr.db_path,"r+b"); if(!f)return ESP_FAIL;
    uint16_t cnt; fread(&cnt,sizeof(cnt),1,f);
    uint16_t nid=cnt+1; fseek(f,0,SEEK_END);
    uint8_t buf[REC_PER_FACE]; memset(buf,0,sizeof(buf));
    *(uint16_t*)buf=nid; strncpy((char*)buf+2,name,31);
    memcpy(buf+2+32,tmpl,TMPL_BYTES); fwrite(buf,1,REC_PER_FACE,f);
    cnt++; fseek(f,0,SEEK_SET); fwrite(&cnt,sizeof(cnt),1,f); fclose(f);
    *oid=nid;
    ESP_LOGI(TAG,"Enrolled id=%d name=\"%s\"",(int)nid,name);
    return ESP_OK;
}

void face_recognition_recognize_async(const uint8_t *rgb, int w, int h, int stride,
                                       const int boxes[][4], const float *scores, int n) {
    if(!g_fr.enabled||!g_fr.initialized||n<=0||!rgb)return;
    hd_result_t det=human_detect_get_results(); if(det.count==0)return;
    hd_box_t *face=&det.boxes[0];
    uint8_t tmpl[TMPL_BYTES];
    extract_aligned_face(rgb,w,h,stride,face,tmpl);
    fr_recog_result_t out; db_match(tmpl,&out);
    if(xSemaphoreTake(g_fr.mutex,pdMS_TO_TICKS(50))==pdTRUE){g_fr.latest=out;xSemaphoreGive(g_fr.mutex);}
}

fr_recog_result_t face_recognition_get_latest(void){
    fr_recog_result_t r; memset(&r,0,sizeof(r));
    if(xSemaphoreTake(g_fr.mutex,pdMS_TO_TICKS(20))==pdTRUE){r=g_fr.latest;xSemaphoreGive(g_fr.mutex);}
    return r;
}

int face_recognition_count(void){
    FILE *f=fopen(g_fr.db_path,"rb"); if(!f)return 0;
    uint16_t c=0; fread(&c,sizeof(c),1,f); fclose(f); return(int)c;
}

int face_recognition_list(uint16_t *ids, char names[][FR_MAX_NAME_LEN], int max) {
    FILE *f=fopen(g_fr.db_path,"rb"); if(!f)return 0;
    uint16_t cnt; fread(&cnt,sizeof(cnt),1,f);
    int n=0; uint8_t buf[REC_PER_FACE];
    for(int i=0;i<cnt&&n<max;i++){
        if(fread(buf,1,REC_PER_FACE,f)<REC_PER_FACE)break;
        uint16_t id=*(uint16_t*)buf; if(id==0)continue;
        ids[n]=id; strncpy(names[n],(char*)buf+2,FR_MAX_NAME_LEN-1);
        names[n][FR_MAX_NAME_LEN-1]='\0'; n++;
    }
    fclose(f); return n;
}

esp_err_t face_recognition_delete(uint16_t id){
    FILE *f=fopen(g_fr.db_path,"r+b"); if(!f)return ESP_FAIL;
    uint16_t cnt; fread(&cnt,sizeof(cnt),1,f);
    uint8_t buf[REC_PER_FACE]; bool ok=false;
    for(int i=0;i<cnt;i++){long pos=ftell(f);
        if(fread(buf,1,REC_PER_FACE,f)<REC_PER_FACE)break;
        if(*(uint16_t*)buf==id){fseek(f,pos,SEEK_SET);memset(buf,0,REC_PER_FACE);fwrite(buf,1,REC_PER_FACE,f);ok=true;break;}
    }
    fclose(f); return ok?ESP_OK:ESP_ERR_NOT_FOUND;
}

esp_err_t face_recognition_clear(void){
    FILE *f=fopen(g_fr.db_path,"wb"); if(!f)return ESP_FAIL;
    uint16_t z=0; fwrite(&z,sizeof(z),1,f); fclose(f);
    return ESP_OK;
}

}

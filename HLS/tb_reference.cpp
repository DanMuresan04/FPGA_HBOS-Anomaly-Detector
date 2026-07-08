#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include "hbos_types.h"
#include "hbos_engine.h"
#include "tb_verdict.h"

void packet_assembler(hls::stream<rx_byte_axis_t>&, hls::stream<sensor_packet_t>&);
void address_engine(hls::stream<sensor_packet_t>&, hls::stream<addr_ctrl_t>&, hls::stream<addr_data_t>&);

static std::string licenta_root() {
    const char* env = std::getenv("LICENTA_ROOT");
    return env ? std::string(env) : std::string("..");
}
static const std::string TRAIN_CSV = licenta_root() + "/datasets/training/datatraining_stripped.csv";
static const std::string TEST_CSV  = licenta_root() + "/datasets/test/datatest_stripped.csv";
static const int   DATA_COLS = 4;
static const int   SPIKE     = 5632;

struct Config {
    const char* name;
    int n;
    int cols[16];
    int w[16];
};

static Config CONFIGS[] = {
    {"all 4  (s1 s2 s3 s4)", 4, {0,1,2,3}, {14,44,159,37}},
    {"no s2  (s1 s3 s4)",    3, {0,2,3},   {14,159,37}},
    {"no s1  (s2 s3 s4)",    3, {1,2,3},   {44,159,37}},
    {"no s3  (s1 s2 s4)",    3, {0,1,3},   {14,44,37}},
    {"no s4  (s1 s2 s3)",    3, {0,1,2},   {14,44,159}},
    {"s3 s4 only",           2, {2,3},     {159,37}},
    {"s3 only",              1, {2},       {159}},
};
static const int N_CONFIGS = sizeof(CONFIGS)/sizeof(CONFIGS[0]);

static void send_byte(hls::stream<rx_byte_axis_t>& rx, ap_uint<8> b, bool last) {
    rx_byte_axis_t beat; beat.data=b; beat.keep=1; beat.strb=1; beat.last=last;
    rx.write(beat);
}
static void send_frame(hls::stream<rx_byte_axis_t>& rx,
                       const long* vals, int n, int active, int opcode, int tlast) {
    send_byte(rx,(ap_uint<8>)n,false); send_byte(rx,(ap_uint<8>)active,false);
    send_byte(rx,(ap_uint<8>)opcode,false); send_byte(rx,(ap_uint<8>)tlast,false);
    send_byte(rx,0,false); send_byte(rx,0,false); send_byte(rx,0,false);
    for (int i=0;i<n;i++){ long v=vals[i];
        send_byte(rx,(ap_uint<8>)(v&0xFF),false);     send_byte(rx,(ap_uint<8>)((v>>8)&0xFF),false);
        send_byte(rx,(ap_uint<8>)((v>>16)&0xFF),false); send_byte(rx,(ap_uint<8>)((v>>24)&0xFF),false); }
    send_byte(rx,FRAME_MAGIC_LO,false); send_byte(rx,FRAME_MAGIC_HI,true);
}
static void run_frame(hls::stream<rx_byte_axis_t>& rx, hls::stream<anomaly_packet_t>& out,
                      const long* vals, int n, int active, int opcode, int tlast) {
    hls::stream<sensor_packet_t> raw;
    hls::stream<addr_ctrl_t> ac; hls::stream<addr_data_t> ad;
    static hls::stream<verdict_beat_t> vout;
    send_frame(rx, vals, n, active, opcode, tlast);
    packet_assembler(rx, raw); address_engine(raw, ac, ad); hbos_engine(ac, ad, vout);
    tb_unpack_verdicts(vout, out);
}

static int stream_csv(hls::stream<rx_byte_axis_t>& rx, hls::stream<anomaly_packet_t>& out,
                      const char* path, const Config& c, int opcode,
                      int* tp=0,int* fp=0,int* fn=0,int* tn=0) {
    std::ifstream f(path);
    if(!f.is_open()){ std::cerr<<"cannot open "<<path<<"\n"; return -1; }
    std::string line; int rows=0;
    while(std::getline(f,line)){
        std::stringstream ss(line); std::string cell;
        long row[16]={0};
        for(int i=0;i<DATA_COLS;i++){ std::getline(ss,cell,','); row[i]=std::stol(cell); }
        std::getline(ss,cell,','); int label=std::stoi(cell);
        long vals[16]={0};
        for(int j=0;j<c.n;j++) vals[j]=row[c.cols[j]];
        run_frame(rx,out,vals,c.n,c.n,opcode,(label==0)?0:1);
        while(!out.empty()){
            int v=(int)out.read().data; bool pred=(v==0x01), act=(label!=0);
            if(tp){ if(pred&&act)(*tp)++; else if(pred&&!act)(*fp)++; else if(!pred&&act)(*fn)++; else (*tn)++; }
        }
        rows++;
    }
    return rows;
}

int main(){
    hls::stream<rx_byte_axis_t> rx; hls::stream<anomaly_packet_t> out;
    printf("=== Reference numbers (merged engine, NR_SENSORS=%d) ===\n", NR_SENSORS);
    printf("train=%s\ntest =%s\nweights(UI)= s1:14 s2:44 s3:159 s4:37  spike=%d\n\n",
           TRAIN_CSV.c_str(), TEST_CSV.c_str(), SPIKE);

    for(int ci=0; ci<N_CONFIGS; ci++){
        Config& c = CONFIGS[ci];

        { long z[16]={0}; run_frame(rx,out,z,1,c.n,OP_RESET,0); while(!out.empty()) out.read(); }

        { long cfg[5]={0,0,0,0,(long)SPIKE};
          for(int i=0;i<c.n;i++){ int word=i>>2, byte=i&3; cfg[word]|=((long)(c.w[i]&0xFF))<<(byte*8); }
          run_frame(rx,out,cfg,5,c.n,OP_CONFIG,0); while(!out.empty()) out.read(); }

        stream_csv(rx,out,TRAIN_CSV.c_str(),c,OP_TRAIN);
        stream_csv(rx,out,TRAIN_CSV.c_str(),c,OP_CALIB);

        long z[16]={0};
        run_frame(rx,out,z,1,c.n,OP_DUMP,0);
        int ack = out.empty()? -1 : (int)out.read().data;
        int telem[5]={-1,-1,-1,-1,-1};
        for(int p=0;p<5;p++){ run_frame(rx,out,z,1,c.n,OP_DUMP,0); telem[p]= out.empty()? -1 : (int)out.read().data; }
        int threshold = (telem[1]>=0)? (telem[1] | (telem[2]<<8) | (telem[3]<<16)) : -1;

        int tp=0,fp=0,fn=0,tn=0;
        stream_csv(rx,out,TEST_CSV.c_str(),c,OP_DETECT,&tp,&fp,&fn,&tn);

        double prec=(tp+fp)?(double)tp/(tp+fp):0, rec=(tp+fn)?(double)tp/(tp+fn):0;
        double f1=(prec+rec)?2*prec*rec/(prec+rec):0;
        printf("--- CONFIG %d: %-22s (active=%d) ---\n", ci, c.name, c.n);
        printf("   ack=0x%02X  threshold=%d\n", ack, threshold);
        printf("   TP=%-5d FP=%-5d FN=%-5d TN=%-5d | P=%.4f R=%.4f F1=%.4f\n\n",
               tp,fp,fn,tn,prec,rec,f1);
    }
    printf("Reference run complete.\n");
    return 0;
}

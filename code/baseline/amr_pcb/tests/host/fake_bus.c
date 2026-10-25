#include "fake_bus.h"
#include "canopen.h"
#include "can_trace.h"
#include "diagnostics.h"
#include "wheel_rpdo_tx.h"
#include "wheel_pdo_config.h"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static FDCAN_GlobalTypeDef registers;
FDCAN_HandleTypeDef hfdcan1 = {&registers};
uint32_t tick, tick_step, fifo_free, bus_off, sdo_ops, fail_sdo, mismatch_sdo;
uint32_t fail_add_mask, fail_request_mask, cancel_hold, restart_sdo, safety_zero_frames;
uint32_t suppress_response, fail_nmt, fail_cancel;
uint32_t injected_abort, bad_width_sdo, initial_type, type_writes, drop_event;
uint32_t identity_accesses;
uint8_t last_sdo_node, last_sdo_sub, last_sdo_write, silent_node, drop_nmt;
uint16_t last_sdo_index;
uint16_t wheel_statusword[3];
uint32_t enable_controlwords, reset_controlwords;
bool persistent_wheel_fault;
FakeSdoHook_t fake_sdo_hook;
FakeRxConsumedHook_t fake_rx_consumed_hook;
uint32_t fake_rx_capacity;
FDCAN_TxHeaderTypeDef tx_headers[2];
uint8_t tx_data[2][4];
static struct { uint32_t id; uint8_t data[64], length, dlc; } rx[128];
static uint32_t rx_head, rx_tail;
static FDCAN_TxEventFifoTypeDef events[8];
static uint32_t event_head, event_tail;
static struct { uint16_t index; uint8_t sub, node, width; uint32_t value; } od[128];
static uint32_t od_count;
static char log_text[65536];
static size_t log_size;
const char *Fake_Log(void) { return log_text; }
void Fake_ClearLog(void) { log_size=0;log_text[0]=0; }
static void Log(const char *text,size_t length)
{
    if(length>=sizeof(log_text)-log_size)length=sizeof(log_text)-log_size-1;
    memcpy(log_text+log_size,text,length);log_size+=length;log_text[log_size]=0;
}

void Fake_Reset(void)
{
    memset(&registers, 0, sizeof(registers));
    Fake_ClearLog();
    memset(od, 0, sizeof(od));
    memset(tx_data, 0, sizeof(tx_data));
    tick=0; tick_step=1; fifo_free=8; bus_off=0; sdo_ops=0;
    fail_sdo=mismatch_sdo=fail_add_mask=fail_request_mask=cancel_hold=restart_sdo=0;
    suppress_response=fail_nmt=fail_cancel=safety_zero_frames=0;
    injected_abort=0x06090011;bad_width_sdo=type_writes=drop_event=0;initial_type=254;
    identity_accesses=0;
    last_sdo_node=last_sdo_sub=last_sdo_write=silent_node=drop_nmt=0;last_sdo_index=0;
    wheel_statusword[1]=wheel_statusword[2]=0x40;
    enable_controlwords=reset_controlwords=0;persistent_wheel_fault=false;
    fake_sdo_hook=NULL;
    fake_rx_consumed_hook=NULL;fake_rx_capacity=128;
    rx_head=rx_tail=event_head=event_tail=od_count=0;
    Canopen_TestResetState(); WheelPdoConfig_TestReset(); WheelRpdoTx_TestReset();
}
uint32_t HAL_GetTick(void) { uint32_t now=tick; tick+=tick_step; return now; }
void HAL_Delay(uint32_t ms) { tick+=ms; }
void Fake_Rx(uint32_t id,const uint8_t *data,uint8_t length)
{
    assert(length<=8);
    Fake_RxWithDlc(id,data,length,length);
}
void Fake_RxWithDlc(uint32_t id,const uint8_t *data,uint8_t length,uint8_t dlc)
{
    assert(length<=64 && dlc<=15);
    assert(fake_rx_capacity>0 && fake_rx_capacity<=128);
    assert(rx_tail-rx_head<fake_rx_capacity); unsigned n=rx_tail++%128;
    rx[n].id=id; rx[n].length=length;rx[n].dlc=dlc;memcpy(rx[n].data,data,length);
}
void Fake_Heartbeat(uint8_t node,uint8_t state) { Fake_Rx(0x700U+node,&state,1); }
void Fake_Tpdo(int32_t left,int32_t right)
{
    uint8_t b[4]; WheelRpdoTx_Encode(left,b); Fake_Rx(0x181,b,4);
    WheelRpdoTx_Encode(right,b); Fake_Rx(0x182,b,4);
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *h,uint32_t f)
{ (void)h;(void)f;return rx_tail-rx_head; }
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h,uint32_t f,FDCAN_RxHeaderTypeDef *r,uint8_t *d)
{
    (void)h;(void)f;if(rx_head==rx_tail)return HAL_ERROR;
    unsigned n=rx_head++%128; memset(r,0,sizeof(*r));r->Identifier=rx[n].id;
    r->DataLength=(uint32_t)rx[n].dlc;memcpy(d,rx[n].data,rx[n].length);
    if(fake_rx_consumed_hook)fake_rx_consumed_hook(r->Identifier);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *h,FDCAN_ProtocolStatusTypeDef *p)
{ (void)h;memset(p,0,sizeof(*p));p->BusOff=bus_off;return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *h,FDCAN_ErrorCountersTypeDef *p)
{ (void)h;memset(p,0,sizeof(*p));return HAL_OK; }
uint32_t HAL_FDCAN_GetError(FDCAN_HandleTypeDef *h) { (void)h;return 0; }
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *h) { (void)h;return fifo_free; }
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxBuffer(FDCAN_HandleTypeDef *h,const FDCAN_TxHeaderTypeDef *p,const uint8_t *d,uint32_t m)
{
    if ((fail_add_mask&m)||(h->Instance->TXBRP&m)) return HAL_ERROR;
    unsigned slot=m==1?0:1;tx_headers[slot]=*p;memcpy(tx_data[slot],d,4);return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_EnableTxBufferRequest(FDCAN_HandleTypeDef *h,uint32_t m)
{
    if(fail_request_mask&m)return HAL_ERROR;
    h->Instance->TXBTO&=~m;h->Instance->TXBCF&=~m;h->Instance->TXBRP|=m;return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *h,uint32_t m)
{
    if(fail_cancel)return HAL_ERROR;
    if(!cancel_hold){h->Instance->TXBCF|=h->Instance->TXBRP&m;h->Instance->TXBRP&=~m;}
    return HAL_OK;
}
void Fake_Complete(uint8_t slot,uint16_t timestamp)
{
    uint32_t m=1U<<slot;assert(registers.TXBRP&m);
    registers.TXBRP&=~m;registers.TXBTO|=m;
    if(drop_event)return;
    assert(event_tail-event_head<8);
    events[event_tail++%8]=(FDCAN_TxEventFifoTypeDef){tx_headers[slot].Identifier,FDCAN_DLC_BYTES_4,slot,timestamp};
    registers.TXEFS=event_tail-event_head;
}
HAL_StatusTypeDef HAL_FDCAN_GetTxEvent(FDCAN_HandleTypeDef *h,FDCAN_TxEventFifoTypeDef *e)
{
    if(event_tail==event_head)return HAL_ERROR;
    *e=events[event_head++%8];h->Instance->TXEFS=event_tail-event_head;return HAL_OK;
}

HAL_StatusTypeDef CanTrace_SubmitTx(FDCAN_HandleTypeDef *h,const FDCAN_TxHeaderTypeDef *p,
    const uint8_t *data,CanTraceTxOwner_t owner,uint8_t node,uint16_t index,uint8_t sub,bool write)
{
    (void)h;(void)owner;(void)index;(void)sub;(void)write;
    if(bus_off)return HAL_ERROR;
    if(p->Identifier==0){
        if(fail_nmt)return HAL_ERROR;
        if(drop_nmt!=data[1])Fake_Heartbeat(data[1],data[0]==0x80?0x7f:5);
        return HAL_OK;
    }
    if(p->Identifier<0x601 || p->Identifier>0x603)return HAL_OK;
    node=(uint8_t)(p->Identifier-0x600);index=(uint16_t)(data[1]|((uint16_t)data[2]<<8));sub=data[3];
    uint8_t reply[8]={0,data[1],data[2],sub,0,0,0,0};uint32_t value=0;
    unsigned found=0;while(found<od_count && !(od[found].node==node&&od[found].index==index&&od[found].sub==sub))found++;
    sdo_ops++;
    last_sdo_node=node;last_sdo_index=index;last_sdo_sub=sub;last_sdo_write=data[0]!=0x40;
    if(index==0x1018)identity_accesses++;
    if(restart_sdo==sdo_ops)Fake_Heartbeat(1,0);
    if(fake_sdo_hook && fake_sdo_hook(node,index,sub,data))return HAL_OK;
    if(suppress_response==sdo_ops || silent_node==node)return HAL_OK;
    /* This simulated device has NO identity object. Count and abort every
     * access, rather than manufacturing IDs to satisfy initialization. */
    if(index==0x1018){reply[0]=0x80;reply[6]=0x02;reply[7]=0x06;}
    else if(fail_sdo==sdo_ops){reply[0]=0x80;for(unsigned i=0;i<4;i++)reply[4+i]=(uint8_t)(injected_abort>>(8*i));}
    else if(data[0]==0x40){
        uint8_t width=found<od_count?od[found].width:4;
        value=found<od_count?od[found].value:0;
        if(index==0x1401 && sub==2 && found==od_count){value=initial_type;width=1;}
        if(index==0x6041){value=node<=2?wheel_statusword[node]:0x40;width=2;}
        if(index==0x603f){value=0;width=2;}
        if(index==0x1001){value=0;width=1;}
        if(mismatch_sdo==sdo_ops)value^=1;
        reply[0]=width==1?0x4f:(width==2?0x4b:0x43);
        if(bad_width_sdo==sdo_ops)reply[0]=0x4f;
        for(unsigned i=0;i<4;i++)reply[4+i]=(uint8_t)(value>>(8*i));
    }else{
        for(unsigned i=0;i<4;i++)value|=(uint32_t)data[4+i]<<(8*i);
        if(found==od_count){assert(od_count<128);od_count++;}
        od[found].node=node;od[found].index=index;od[found].sub=sub;od[found].value=value;
        od[found].width=data[0]==0x2f?1:(data[0]==0x2b?2:4);reply[0]=0x60;
        if(index==0x6040 && node<=2){
            if(value==0x000f || value==0x002f)enable_controlwords++;
            if(value==0x0086){
                reset_controlwords++;
                if(!persistent_wheel_fault)wheel_statusword[node]=0x40;
            }
        }
        if(index==0x1401 && sub==2){assert(value==254);type_writes++;}
        if(index==0x60ff && value==0){
            /* Any safety zero sent while a nonzero RPDO remains is a bug. */
            assert(registers.TXBRP==0);safety_zero_frames++;
        }
    }
    Fake_Rx(0x580U+node,reply,8);return HAL_OK;
}

/* UART and trace sinks are the only non-transport mocks. Real CANopen, Drive,
 * safety, motion math, TPDO pairing and ODOM objects are linked by the runner. */
bool JetsonTx_Printf(JetsonTxPriority_t p,const char *fmt,...)
{
    (void)p;char text[512];va_list ap;va_start(ap,fmt);int n=vsnprintf(text,sizeof(text),fmt,ap);va_end(ap);
    assert(n>=0 && (unsigned)n<=JETSON_TX_FRAME_CAPACITY);Log(text,(size_t)n);return true;
}
bool JetsonTx_Send(JetsonTxPriority_t p,const char *data,uint16_t length)
{ (void)p;Log(data,length);return true; }
void uart_printf(const char *fmt,...)
{
    char text[512];va_list ap;va_start(ap,fmt);int n=vsnprintf(text,sizeof(text),fmt,ap);va_end(ap);
    assert(n>=0 && n<(int)sizeof(text));Log(text,(size_t)n);
}
void JetsonTx_Process(void) {}
JetsonTxStats_t JetsonTx_GetStats(void) { JetsonTxStats_t stats={0};return stats; }
void uart_printf_priority(JetsonTxPriority_t p,const char *fmt,...) { (void)p;(void)fmt; }
const char *FDCAN_HandleName(FDCAN_HandleTypeDef *h) { (void)h;return "FDCAN1"; }
const char *hal_status_name(HAL_StatusTypeDef s) { (void)s;return "MOCK"; }
void CAN_Print_Rx_Frame(FDCAN_HandleTypeDef *h,const FDCAN_RxHeaderTypeDef *r,const uint8_t *d,uint8_t l)
{ (void)h;(void)r;(void)d;(void)l; }
bool CAN1_Print_Status(const char *t,CAN1_DiagStatus_t *s) { (void)t;memset(s,0,sizeof(*s));return true; }


#define AP_SERIALMANAGER_OPEN_MV_BAUD         115200    //串口波特率
#define AP_SERIALMANAGER_OPENMV_BUFSIZE_RX        64    //串口接受缓存大小
#define AP_SERIALMANAGER_OPENMV_BUFSIZE_TX        64    //串口发送缓存大小

#include "AP_OpenMV.h"

extern const AP_HAL::HAL& hal;

//constructor
AP_OpenMV::AP_OpenMV(void)
{
    _port = NULL;   //默认OpenMV串口为空
    _step = 0;      //默认解析帧的步骤为0
    _singleton = this;
}

// init - perform require initialisation including detecting which protocol to use
void AP_OpenMV::init(const AP_SerialManager& serial_manager)
{
    // check for DEVO_DPort
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_OPEN_MV, 0))) {    //查找飞控上的串口里是否有连接OpenMV的
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);      //关闭流控制
        // initialise uart
        //打开串口
        _port->begin(AP_SERIALMANAGER_OPEN_MV_BAUD, AP_SERIALMANAGER_OPENMV_BUFSIZE_RX, AP_SERIALMANAGER_OPENMV_BUFSIZE_TX);
    }
}

bool AP_OpenMV::update()
{
    if(_port == NULL)
    {
        flag = 0;
        return false;
    }
    flag = 1;    

    int16_t numc = _port->available();  //读取的字节数
    uint8_t data;
    uint8_t checksum = 0;   //接收数据的校验和

    for (int16_t i = 0; i < numc; i++) {
        data = _port->read();

        switch(_step) {
        case 0:
            if(data == 0xA5)    //帧头1
                _step = 1;
            break;

        case 1:
            if(data == 0x5A)    //帧头2
                _step = 2;
            else
                _step = 0;
            break;

        case 2:
            _cx_temp = data;    //x轴坐标
            _step = 3;
            break;

        case 3:
            _cy_temp = data;    //y轴坐标
            _step = 4;
            break;

        case 4:
            _step = 0;
            checksum = _cx_temp + _cy_temp;
            if(checksum == data) {
                cx = _cx_temp;
                cy = _cy_temp;
                last_frame_ms = AP_HAL::millis();   //记录这一帧成功解析的时间
                return true;
            }
            break;

        default:
            _step = 0;
        }
    }

    return false;
}

AP_OpenMV *AP_OpenMV::_singleton;

namespace AP {

AP_OpenMV *openmv()
{
    return AP_OpenMV::get_singleton();
}

}

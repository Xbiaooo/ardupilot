
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
}

// init - perform require initialisation including detecting which protocol to use
void AP_OpenMV::init(const AP_SerialManager& serial_manager)
{
    // check for Openmv_Port
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
        return false;

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
            if (data == 0x00)  //x正负
            {
                cx_flag = 1.0f;
                _step = 3;
            }                       
            else if (data == 0xFF)
            {
                cx_flag = -1.0f;
                _step = 3;
            }                
            else
            {
                _step = 0;
            }
            break;

        case 3:
            cx_int_temp = data;    //x轴坐标整数值
            _step = 4;
            break;

        case 4:
            cx_dec_temp = data;   //x轴坐标小数值
            _step = 5;
            break;

        case 5:
            if (data == 0x00)  //y正负
            {
                cy_flag = 1.0f;
                _step = 6;
            }                       
            else if (data == 0xFF)
            {
                cy_flag = -1.0f;
                _step = 6;
            }                
            else
            {
                _step = 0;
            }
            break;

        case 6:
            cy_int_temp = data;    //y轴坐标整数值
            _step = 7;
            break;

        case 7:
            cy_dec_temp = data;   //y轴坐标小数值
            _step = 8;
            break;

        case 8:
            cz_int_temp = data;    //z轴坐标整数值
            _step = 9;
            break;

        case 9:
            cz_dec_temp = data;   //z轴坐标小数值
            _step = 10;
            break;

        case 10:
            _step = 0;
            checksum = cx_dec_temp + cy_dec_temp;
            if(checksum == data) {
                cx = cx_flag * (cx_int_temp + 0.01 * cx_dec_temp);
                cy = cy_flag * (cy_int_temp + 0.01 * cy_dec_temp);
                cz = cz_int_temp + 0.01 * cz_dec_temp;
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

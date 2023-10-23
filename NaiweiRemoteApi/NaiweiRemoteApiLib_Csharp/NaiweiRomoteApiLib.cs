using NaiweiRobot;
using NModbus;
using NModbus.Serial;
using System;
using System.ComponentModel;
using System.IO.Ports;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Security.Cryptography;
using System.Security.Principal;

namespace NaiweiRemoteApiLib
{
    public class NaiweiRomoteApiLib
    {
        private TcpClient client_;
        private IModbusMaster master_;
        private const byte id_ = 1;//从站编号，从1开始
        private string ip_ = "192.168.0.2";//通信地址
        private int port_ = 502;//通信端口

        private SerialPort sport_;
        private byte commtype_;//通信方式 0TCP，1RTU
        private NaiweiRemoteApiTool tool_ = new NaiweiRemoteApiTool();

        /// <summary>
        /// 与机器人Modbus服务器创建TCP连接
        /// </summary>
        /// <param name="ip">Modbus服务器网址</param>
        /// <param name="port">Modbus服务器端口</param>
        /// <returns>连接结果</returns>
        public bool Connect(string ip, int port=502)
        {
            if (client_ != null)
                if (client_.Connected)
                    return true;

            ip_ = ip;
            port_ = port;

            client_ = new TcpClient(ip_, port_);
            var factory = new ModbusFactory();
            master_ = factory.CreateMaster(client_);
            master_.Transport.ReadTimeout = 1000;//超时
            master_.Transport.Retries = 5;//重试
            commtype_ = 0;//TCP方式

            return client_.Connected;
        }

        public bool Connect(string port="COM1", int baudrate=9600, Parity parity = Parity.Odd)
        {
            if (sport_ != null)
                if (sport_.IsOpen)
                    return true;

            sport_ = new SerialPort(port);
            sport_.BaudRate = baudrate;
            sport_.Parity = parity;
            sport_.DataBits = 8;
            sport_.StopBits = StopBits.One;
            sport_.Open();

            var factory = new ModbusFactory();
            master_ = factory.CreateRtuMaster(sport_);
            master_.Transport.ReadTimeout = 1000;//超时
            master_.Transport.Retries = 5;//重试
            commtype_ = 1;//RTU方式

            return sport_.IsOpen;
        }

        /// <summary>
        /// 与机器人Modbus服务器断开连接
        /// </summary>
        /// <returns>断连结果</returns>
        public bool DisConnect()
        {
            if (commtype_ == 0)
            {
                if (client_ == null) return true;
                if (!client_.Connected) return true;

                client_.Close();
                return !client_.Connected;
            }
            else
            {
                if (sport_ == null) return true;
                if (!sport_.IsOpen) return true;

                sport_.Close();
                return !sport_.IsOpen;
            }
        }

        /// <summary>
        /// 查询与机器人Modbus服务器的连接状态
        /// </summary>
        /// <returns>连接状态</returns>
        public bool IsConnected()
        {
            return commtype_==0? client_.Connected: sport_.IsOpen;
        }

        /// <summary>
        /// 设置使能效果
        /// </summary>
        /// <param name="enable">使能效果</param>
        public void Power(bool enable = true)
        {
            //若未连接，直接退出
            if (!client_.Connected) return;

            //查询使能状态
            var reg = tool_.AddressConvert(Address.PowerOk);
            var res = master_.ReadHoldingRegisters(id_, reg[0], 1);
            var sign = tool_.GetBit(res[0], reg[1], out _);

            //若当前使能状态和期望使能操作已一致，直接退出
            if (sign == enable) return;

            //写入使能效果
            reg = tool_.AddressConvert(Address.PowerSignal);
            res = master_.ReadHoldingRegisters(id_, reg[0], 1);
            tool_.SetBit(ref res[0], reg[1], enable);
            master_.WriteMultipleRegisters(id_, reg[0], res);
        }

        /// <summary>
        /// 设置操作模式
        /// </summary>
        /// <param name="mode">目标操作模式</param>
        /// <returns>操作结果状态</returns>
        public bool SetOpMode(OpModeType mode)
        {
            //若未连接，直接退出
            bool sign = false;
            if (!client_.Connected) return sign;

            //计算源数据
            short[] src = new short[1] { (short)mode };

            //输入数据类型转换
            var dst = tool_.T2Ushort<short>(ref src, out sign);
            if(!sign) return sign;

            //写入目标数据
            var reg=tool_.AddressConvert(Address.OpMode);
            master_.WriteMultipleRegisters(id_, reg[0], dst);

            return true;
        }

        /// <summary>
        /// 查询操作模式
        /// </summary>
        /// <param name="sign">操作结果状态</param>
        /// <returns>当前操作模式</returns>
        public OpModeType GetOpMode(out bool sign)
        {
            //若未连接，直接退出
            sign = false;
            if (!client_.Connected) return OpModeType.Manual;

            //查询目标寄存器
            var reg = tool_.AddressConvert(Address.OpMode);
            var res = master_.ReadHoldingRegisters(id_, reg[0], 1);

            //输出数据类型转换
            var tmp = tool_.Ushort2T<short>(ref res, out sign);
            if (!sign) return OpModeType.Manual;

            //计算返回值
            var ret = (OpModeType)tmp[0];

            //sign = true;
            return ret;
        }

        /// <summary>
        /// 执行手动模式下的jog运动
        /// </summary>
        /// <param name="index">选定轴的索引，从0开始</param>
        /// <param name="direction">jog运动方向 1正向，-1负向</param>
        /// <param name="enable">运动触发 true启动，false停止</param>
        public void Jog(ushort index, short direction, bool enable=true)
        {
            //若未连接，直接退出
            if (!client_.Connected) return;

            //计算内存地址
            Address address;
            switch(direction)
            {
                case 1: address =Address.JogPlus; break;
                case -1: address =Address.JogMinus; break;
                default: return;
            }

            //读取目标寄存器
            var reg = tool_.AddressConvert(address);
            reg[0] += (ushort)(index >> 1);//计算第index个端口的起始寄存器地址
            var res = master_.ReadHoldingRegisters(id_, reg[0], 1);

            //信号合并
            var src = tool_.Ushort2T<bool>(ref res, out _);
            src[index % 2] = enable;
            res = tool_.T2Ushort<bool>(ref src, out _);

            //写入目标数据
            master_.WriteMultipleRegisters(id_, reg[0], res);

            return;
        }

        /// <summary>
        /// 执行手动模式下的定位运动
        /// </summary>
        /// <param name="type">目标点类型</param>
        /// <param name="index">目标点索引，从0开始</param>
        /// <param name="enable">运动触发 true启动，false停止</param>
        public void Move(VarType type, ushort index, bool enable)
        {
            //若未连接，直接退出
            if (!client_.Connected) return;

            //选中待操作的坐标
            switch (type)
            {
                case VarType.LJ: index += 1000; break;
                case VarType.LP: index += 2000; break;
                case VarType.GJ: index += 3000; break;
                case VarType.GP: index += 4000; break;
                default: return;
            }
            var reg = tool_.AddressConvert(Address.RemotePosSel);
            master_.WriteSingleRegister(id_, reg[0], index);

            //触发运动效果
            reg = tool_.AddressConvert(Address.RemotePosLoc);
            var res = master_.ReadHoldingRegisters(id_, reg[0], 1)[0];
            tool_.SetBit(ref res, reg[1], enable);
            master_.WriteSingleRegister(id_, reg[0], res);
        }

        /// <summary>
        /// 任务文件切换
        /// </summary>
        /// <param name="num">目标任务的文件编号</param>
        /// <returns>操作结果状态</returns>
        public bool Task(ushort num)
        {
            //若未连接，直接退出
            if (!client_.Connected) return false;

            //若非手动模式，直接退出
            var reg = tool_.AddressConvert(Address.OpMode);
            var res = master_.ReadHoldingRegisters(id_, reg[0], 1)[0];
            if (res != 0) return false;
            
            //选中目标任务文件
            reg=tool_.AddressConvert(Address.TaskNumNow);
            master_.WriteSingleRegister(id_, reg[0], num);

            //选择读取目标文件
            reg=tool_.AddressConvert(Address.TaskType);
            master_.WriteSingleRegister(id_, reg[0], 0);

            //触发更改任务文件编号
            reg = tool_.AddressConvert(Address.TaskNumMod);
            res = master_.ReadHoldingRegisters(id_, reg[0], 1)[0];
            tool_.SetBit(ref res, reg[1], true);
            master_.WriteSingleRegister(id_, reg[0], res);

            //触发读取任务文件内容
            reg = tool_.AddressConvert(Address.TaskSignal);
            res = master_.ReadHoldingRegisters(id_, reg[0], 1)[0];
            tool_.SetBit(ref res, reg[1], true);
            master_.WriteSingleRegister(id_, reg[0], res);

            //查询当前执行的任务文件编号
            reg = tool_.AddressConvert(Address.TaskNumExe);
            res = master_.ReadHoldingRegisters(id_, reg[0], 1)[0];

            return res == num ? true : false;
        }

        /// <summary>
        /// 设置自动执行模式
        /// </summary>
        /// <param name="value">执行模式，0循环，1单次，3单步</param>
        /// <returns>操作结果状态</returns>
        public bool SetAutoMode(ushort value)
        {
            //若未连接，直接退出
            if (!client_.Connected) return false;

            //写入目标数据
            ushort[] src = new ushort[1]{ value};
            var reg= tool_.AddressConvert(Address.AutoMode);
            master_.WriteMultipleRegisters(id_, reg[0], src);

            return true;
        }

        /// <summary>
        /// 查询自动执行模式
        /// </summary>
        /// <returns>执行模式，0循环，1单次，3单步</returns>
        public ushort GetAutoMode()
        {
            //若未连接，直接退出
            if(!client_.Connected) return 0;

            //写入目标寄存器
            var reg =tool_.AddressConvert(Address.AutoMode);
            var res= master_.ReadHoldingRegisters(id_, reg[0], 1)[0];

            return res;
        }

        /// <summary>
        /// 查询自动状态
        /// </summary>
        /// <returns>状态标志 0停止，1启动，2暂停，3运行状态；4暂停状态；5停止状态</returns>
        public ushort GetAutoStatus()
        {
            //若未连接，直接退出
            if (!client_.Connected) return 0;

            //写入目标寄存器
            var reg = tool_.AddressConvert(Address.AutoStatus);
            var res = master_.ReadHoldingRegisters(id_, reg[0], 1)[0];

            return res;
        }

        /// <summary>
        /// 自动启动
        /// </summary>
        /// <param name="enable">是否启动</param>
        /// <returns>操作结果</returns>
        public bool MotionStart(bool enable)
        {
            //若未连接，直接退出
            if (!client_.Connected) return false;

            //读取自动流程
            var reg = tool_.AddressConvert(Address.AutoStart);
            var res = master_.ReadHoldingRegisters(id_, reg[0], 1)[0];

            //位操作自动start
            tool_.SetBit(ref res, reg[1], enable);
            master_.WriteSingleRegister(id_, reg[0], res);

            return true;
        }

        /// <summary>
        /// 自动暂停
        /// </summary>
        /// <param name="enable">是否暂停</param>
        /// <returns>操作结果</returns>
        public bool MotionPause(bool enable)
        {
            //若未连接，直接退出
            if (!client_.Connected) return false;

            //读取自动流程
            var reg = tool_.AddressConvert(Address.AutoPause);
            var res = master_.ReadHoldingRegisters(id_, reg[0], 1)[0];

            //位操作自动pause
            tool_.SetBit(ref res, reg[1], enable);
            master_.WriteSingleRegister(id_, reg[0], res);

            return true;
        }

        /// <summary>
        /// 自动停止
        /// </summary>
        /// <param name="enable">是否停止</param>
        /// <returns>操作结果</returns>
        public bool MotionStop(bool enable)
        {
            //若未连接，直接退出
            if (!client_.Connected) return false;

            //读取自动流程
            var reg = tool_.AddressConvert(Address.AutoStop);
            var res = master_.ReadHoldingRegisters(id_, reg[0], 1)[0];

            //位操作自动stop
            tool_.SetBit(ref res, reg[1], enable);
            master_.WriteSingleRegister(id_, reg[0], res);

            return true;
        }

        /// <summary>
        /// 查询报错编号
        /// </summary>
        /// <returns>报错编号</returns>
        public int GetError()
        {
            //若未连接，直接退出
            if (!client_.Connected) return 0;

            //获取报错码
            var reg = tool_.AddressConvert(Address.AlarmCode);
            var res = master_.ReadHoldingRegisters(id_, reg[0], 1);

            //输出数据类型转换
            var ret = tool_.Ushort2T<short>(ref res, out _)[0];

            return ret;
        }

        /// <summary>
        /// 设置报错复位
        /// </summary>
        /// <param name="enable">执行效果</param>
        public void ResetError(bool enable = true)
        {
            //若未连接，直接退出
            if (!client_.Connected) return;

            //写入复位效果
            var reg = tool_.AddressConvert(Address.AlarmReset);
            var res = master_.ReadHoldingRegisters(id_, reg[0], 1);
            tool_.SetBit(ref res[0], reg[1], enable);
            master_.WriteMultipleRegisters(id_, reg[0], res);
        }

        /// <summary>
        /// 设置全局速度
        /// </summary>
        /// <param name="value">速度百分比，取值1~100</param>
        /// <returns>写结果</returns>
        public bool SetGlobalSpeed(ushort value)
        {
            //若未连接，直接退出
            if (!client_.Connected) return false;

            //写入速度百分比
            var reg = tool_.AddressConvert(Address.SpeedGet);
            master_.WriteSingleRegister(id_, reg[0], value);

            //触发修改信号(读取->位操作->写入)
            reg = tool_.AddressConvert(Address.SpeedSet);
            var res = master_.ReadHoldingRegisters(id_, reg[0], 1)[0];
            tool_.SetBit(ref res, reg[1], true);
            master_.WriteSingleRegister(id_, reg[0], res);

            return true;
        }

        /// <summary>
        /// 查询全局速度
        /// </summary>
        /// <returns>速度百分比，取值1~100</returns>
        public ushort GetGlobalSpeed()
        {
            //若未连接，直接退出
            if (!client_.Connected) return 0;

            //读取速度百分比
            var reg = tool_.AddressConvert(Address.SpeedGet);
            var res = master_.ReadHoldingRegisters(id_, reg[0], 1)[0];

            return res;
        }

        /// <summary>
        /// 设置坐标系类型
        /// </summary>
        /// <param name="cs">目标坐标系 0Jcs,1Wcs</param>
        /// <returns>操作结果状态</returns>
        public bool SetCs(CsType cs)
        {
            //若未连接，直接退出
            if (!client_.Connected) return false;

            //写入坐标类型
            var reg = tool_.AddressConvert(Address.CsType);
            master_.WriteSingleRegister(id_, reg[0], (ushort)cs);

            return true;
        }

        /// <summary>
        /// 查询坐标系类型
        /// </summary>
        /// <returns>当前坐标系 0Jcs,1Wcs</returns>
        public CsType GetCs()
        {
            //若未连接，直接退出
            if (!client_.Connected) return 0;

            //读取坐标类型
            var reg = tool_.AddressConvert(Address.CsType);
            var res = master_.ReadHoldingRegisters(id_, reg[0], 1)[0];
            if (res > 1) res = 0;

            return (CsType)res;
        }

        /// <summary>
        /// 查询当前关节坐标
        /// </summary>
        /// <returns>关节坐标值，共6+3轴</returns>
        public float[] GetCurJPos()
        {
            //若未连接，直接退出
            if (!client_.Connected) return new float[1];

            //读取坐标值
            var reg = tool_.AddressConvert(Address.CurrentPosJ);
            var res = master_.ReadInputRegisters(id_, reg[0], 9<<1);
            
            //输出数据类型转换
            float[] ret = tool_.Ushort2T<float>(ref res, out _);

            return ret;
        }

        /// <summary>
        /// 查询当前笛卡尔坐标
        /// </summary>
        /// <returns>笛卡尔坐标值，共6+3轴</returns>
        public float[] GetCurCPos()
        {
            //若未连接，直接退出
            if (!client_.Connected) return new float[1];

            //读取坐标值
            var reg = tool_.AddressConvert(Address.CurrentPosC);
            var res = master_.ReadInputRegisters(id_, reg[0], 9 << 1);

            //输出数据类型转换
            float[] ret = tool_.Ushort2T<float>(ref res, out _);

            return ret;
        }

        /// <summary>
        /// 设置目标点坐标
        /// </summary>
        /// <param name="type">目标点类型</param>
        /// <param name="index">目标点索引，从0开始</param>
        /// <param name="value">目标点坐标，共6+3轴</param>
        /// <returns>操作结果状态</returns>
        public bool SetPos(VarType type, ushort index, float[] value)
        {
            //若未连接，直接退出
            bool sign = false;
            if (!client_.Connected) return sign;

            //选中待读取的坐标
            switch (type)
            {
                case VarType.LJ: index += 1000; break;
                case VarType.LP: index += 2000; break;
                case VarType.GJ: index += 3000; break;
                case VarType.GP: index += 4000; break;
                default: return sign;
            }
            var reg = tool_.AddressConvert(Address.RemotePosSel);
            master_.WriteSingleRegister(id_, reg[0], index);

            //写入坐标
            var src = tool_.T2Ushort<float>(ref value, out sign);
            if (!sign) return sign;
            reg = tool_.AddressConvert(Address.RemotePosWo);
            master_.WriteMultipleRegisters(id_, reg[0], src);

            //触发更改
            reg = tool_.AddressConvert(Address.RemotePosMod);
            var res = master_.ReadHoldingRegisters(id_, reg[0], 1);
            tool_.SetBit(ref res[0], reg[1], true);
            master_.WriteMultipleRegisters(id_, reg[0], res);

            return true;
        }

        /// <summary>
        /// 查询目标点坐标
        /// </summary>
        /// <param name="type">目标点类型</param>
        /// <param name="index">目标点索引，从0开始</param>
        /// <param name="sign">操作结果状态</param>
        /// <returns>目标点坐标，共6+3轴</returns>
        public float[] GetPos(VarType type, ushort index, out bool sign)
        {
            //若未连接，直接退出
            sign = false;
            if (!client_.Connected) return new float[9];

            //选中待读取的坐标
            switch (type)
            {
                case VarType.LJ: index += 1000; break;
                case VarType.LP: index += 2000; break;
                case VarType.GJ: index += 3000; break;
                case VarType.GP: index += 4000; break;
                default: return new float[9];
            }
            var reg = tool_.AddressConvert(Address.RemotePosSel);
            master_.WriteSingleRegister(id_, reg[0], index);

            //读取坐标
            reg = tool_.AddressConvert(Address.RemotePosRo);
            var len = sizeof(float) >> 1;
            var num = (ushort)(len * 9);
            var res = master_.ReadHoldingRegisters(id_, reg[0], num);

            //输出数据类型转换
            float[] ret = tool_.Ushort2T<float>(ref res, out sign);

            return ret;
        }

        /// <summary>
        /// 设置多个连续的IO端口
        /// </summary>
        /// <param name="index">端口起始索引，从0开始</param>
        /// <param name="value">端口数据</param>
        /// <param name="type">端口类型</param>
        /// <returns>操作结果状态</returns>
        public bool SetIo(ushort index, bool[] value, IOType type=IOType.DO)
        {
            //若未连接，直接退出
            bool sign =false;
            if (!client_.Connected) return sign;

            //不支持的端口类型，直接退出
            if (type!=IOType.DO) return sign;

            //查询端口对应寄存器的当前数据
            var num=value.Length;
            var reg = tool_.AddressConvert(Address.DigOut);
            reg[0] += (ushort)(index >> 1);//计算第index个端口的起始寄存器地址
            var regnum = Math.Ceiling((index + num) * 0.5) - Math.Floor(index * 0.5);//计算寄存器个数
            var res = master_.ReadHoldingRegisters(id_, reg[0], (ushort)regnum);

            //比较数据类型转换
            var src = tool_.Ushort2T<bool>(ref res, out sign);
            if (!sign) return sign;

            //数据合并(仅修改从index开始的num个bool元素，其余bool元素保持原值)
            Buffer.BlockCopy(value, 0, src, index % 2, num);

            //输出数据类型转换
            res = tool_.T2Ushort<bool>(ref src, out sign);
            if (!sign) return sign;

            //写入目标数据
            master_.WriteMultipleRegisters(id_, reg[0], res);

            return true;
        }

        /// <summary>
        /// 查询多个连续的IO端口
        /// </summary>
        /// <param name="type">端口类型</param>
        /// <param name="index">端口起始索引，从0开始</param>
        /// <param name="num">端口个数</param>
        /// <param name="sign">操作结果状态</param>
        /// <returns>所查询的目标端口数据</returns>
        public bool[] GetIo(IOType type, ushort index, ushort num, out bool sign)
        {
            //若未连接，直接退出
            sign = false;
            if (!client_.Connected) return new bool[num];

            //读取目标寄存器
            Address address;
            switch(type)
            {
                case IOType.DI:
                    address = Address.DigIn;
                    break;
                case IOType.DO:
                    address = Address.DigOut;
                    break;
                default:
                    return new bool[num];
            }
            var reg = tool_.AddressConvert(address);
            reg[0] += (ushort)(index >> 1);//计算第index个端口的起始寄存器地址
            var regnum = Math.Ceiling((index + num) * 0.5) - Math.Floor(index * 0.5);//计算寄存器个数
            var res = master_.ReadHoldingRegisters(id_, reg[0], (ushort)regnum);

            //输出数据类型转换
            var tmp = tool_.Ushort2T<bool>(ref res, out sign);
            if (!sign) return new bool[num];
            
            //输出个数截取
            bool[] ret = new bool[num];
            Buffer.BlockCopy(tmp, index % 2, ret, 0, num);

            //sign = true;
            return ret;
        }

        /// <summary>
        /// 设置多个指定的IO端口为目标值
        /// </summary>
        /// <param name="indexs">指定的端口索引数组，从0开始</param>
        /// <param name="value">目标值</param>
        /// <returns>操作结果</returns>
        public bool SetFixDo(int[] indexs, bool value=true)
        {
            //若未连接，直接退出
            bool sign = false;
            if (!client_.Connected) return sign;

            foreach(int index in indexs)
            {
                //读取目标寄存器
                var reg = tool_.AddressConvert(Address.DigOut);
                reg[0] += (ushort)(index >> 1);//计算第index个端口的起始寄存器地址
                var res = master_.ReadHoldingRegisters(id_, reg[0], 1);

                //比较数据类型转换
                var tmp = tool_.Ushort2T<bool>(ref res, out sign);
                if (!sign) return sign;

                //数据合并
                tmp[index%2]=value;

                //输出数据类型转换
                res = tool_.T2Ushort<bool>(ref tmp, out sign);
                if (!sign) return sign;

                //写入目标数据
                master_.WriteMultipleRegisters(id_, reg[0], res);
            }

            return true;
        }

        /// <summary>
        /// 查询多个指定的IO端口
        /// </summary>
        /// <param name="index">指定的端口索引数组，从0开始</param>
        /// <param name="sign">操作结果状态</param>
        /// <returns>所查询的目标端口数据数组</returns>
        public bool[] GetFixDo(int[] index, out bool sign)
        {
            //若未连接，直接退出
            sign = false;
            var num = index.Length;
            if (!client_.Connected) return new bool[num];

            bool[] ret = new bool[num];
            for (int i=0; i<num; i++)
            {
                //读取目标寄存器
                var reg = tool_.AddressConvert(Address.DigOut);
                reg[0] += (ushort)(index[i] >> 1);//计算第index个端口的起始寄存器地址
                var res = master_.ReadHoldingRegisters(id_, reg[0], 1);

                //比较数据类型转换
                var src = tool_.Ushort2T<bool>(ref res, out sign);
                if (!sign) return ret;

                //数据输出
                ret[i] = src[index[i] % 2];
            }

            return ret;
        }

        /// <summary>
        /// 设置多个连续的布尔变量
        /// </summary>
        /// <param name="index">变量起始索引，从0开始</param>
        /// <param name="value">变量数值</param>
        /// <param name="scope">变量作用域</param>
        /// <returns>操作结果状态</returns>
        public bool SetBool(ushort index, bool[] value, ScopeType scope = ScopeType.Global)
        {
            //若未连接，直接退出
            bool sign = false;
            if (!client_.Connected) return sign;

            //查询变量对应寄存器的当前数据
            var num = value.Length;
            var reg = tool_.AddressConvert(Address.VarGB);
            reg[0] += (ushort)(index >> 1);//计算第index个变量的起始寄存器地址
            var regnum = Math.Ceiling((index + num) * 0.5) - Math.Floor(index * 0.5);//计算寄存器个数
            var res = master_.ReadHoldingRegisters(id_, reg[0], (ushort)regnum);

            //比较数据类型转换
            var src = tool_.Ushort2T<bool>(ref res, out sign);
            if (!sign) return sign;

            //数据合并(仅修改从index开始的num个bool元素，其余bool元素保持原值)
            Buffer.BlockCopy(value, 0, src, index % 2, num);

            //输出数据类型转换
            res = tool_.T2Ushort<bool>(ref src, out sign);
            if (!sign) return sign;

            //写入目标数据
            master_.WriteMultipleRegisters(id_, reg[0], res);

            return true;
        }

        /// <summary>
        /// 查询多个连续的布尔变量
        /// </summary>
        /// <param name="index">变量起始索引，从0开始</param>
        /// <param name="num">变量个数</param>
        /// <param name="sign">操作结果状态</param>
        /// <param name="scope">变量作用域</param>
        /// <returns>所查询的目标布尔变量数组</returns>
        public bool[] GetBool(ushort index, ushort num, out bool sign, ScopeType scope = ScopeType.Global)
        {
            //若未连接，直接退出
            sign = false;
            if (!client_.Connected) return new bool[num];

            //读取目标寄存器
            var reg = tool_.AddressConvert(Address.VarGB);
            reg[0] += (ushort)(index >> 1);//计算第index个变量的起始寄存器地址
            var regnum = Math.Ceiling((index + num) * 0.5) - Math.Floor(index * 0.5);//计算寄存器个数
            var res = master_.ReadHoldingRegisters(id_, reg[0], (ushort)regnum);

            //输出数据类型转换
            var tmp = tool_.Ushort2T<bool>(ref res, out sign);
            if (!sign) return new bool[num];

            //输出个数截取
            bool[] ret = new bool[num];
            Buffer.BlockCopy(tmp, index % 2, ret, 0, num);

            //sign = true;
            return ret;
        }

        /// <summary>
        /// 设置多个连续的整型变量
        /// </summary>
        /// <param name="index">变量起始索引，从0开始</param>
        /// <param name="value">变量数值</param>
        /// <param name="scope">变量作用域</param>
        /// <returns>操作结果状态</returns>
        public bool SetInt(ushort index, short[] value, ScopeType scope = ScopeType.Global)
        {
            //若未连接，直接退出
            bool sign = false;
            if (!client_.Connected) return sign;

            //输入数据类型转换
            ushort[] data = tool_.T2Ushort<short>(ref value, out sign);
            if (!sign) return sign;

            //写入目标变量
            var reg = tool_.AddressConvert(Address.VarGI);
            var len = sizeof(short) >> 1;
            reg[0] += (ushort)(len * index);
            master_.WriteMultipleRegisters(id_, reg[0], data);

            return true;
        }

        /// <summary>
        /// 查询多个连续的整型变量
        /// </summary>
        /// <param name="index">变量起始索引，从0开始</param>
        /// <param name="num">变量个数</param>
        /// <param name="sign">操作结果状态</param>
        /// <param name="scope">变量作用域</param>
        /// <returns>所查询的目标整型变量数组</returns>
        public short[] GetInt(ushort index, ushort num, out bool sign, ScopeType scope = ScopeType.Global)
        {
            //若未连接，直接退出
            sign = false;
            if (!client_.Connected) return new short[num];

            //读取目标变量
            var reg = tool_.AddressConvert(Address.VarGI);
            var len = sizeof(short) >> 1;
            reg[0] += (ushort)(len * index);
            num *= (ushort)len;
            var res = master_.ReadHoldingRegisters(id_, reg[0], num);

            //输出数据类型转换
            short[] ret = tool_.Ushort2T<short>(ref res, out sign);

            return ret;
        }

        /// <summary>
        /// 设置多个连续的浮点变量
        /// </summary>
        /// <param name="index">变量起始索引，从0开始</param>
        /// <param name="value">变量数值</param>
        /// <param name="scope">变量作用域</param>
        /// <returns>操作结果</returns>
        public bool SetReal(ushort index, float[] value, ScopeType scope = ScopeType.Global)
        {
            //若未连接，直接退出
            bool sign = false;
            if (!client_.Connected) return sign;

            //输入数据类型转换
            ushort[] data = tool_.T2Ushort<float>(ref value, out sign);
            if (!sign) return sign;

            //写入目标变量
            var reg = tool_.AddressConvert(Address.VarGF);
            var len = sizeof(float) >> 1;
            reg[0] += (ushort)(len * index);
            master_.WriteMultipleRegisters(id_, reg[0], data);

            return true;
        }

        /// <summary>
        /// 查询多个连续的浮点变量
        /// </summary>
        /// <param name="index">变量起始索引，从0开始</param>
        /// <param name="num">变量个数</param>
        /// <param name="sign">操作结果状态</param>
        /// <param name="scope">变量作用域</param>
        /// <returns>所查询的目标浮点变量数组</returns>
        public float[] GetReal(ushort index, ushort num, out bool sign, ScopeType scope = ScopeType.Global)
        {
            //若未连接，直接退出
            sign = false;
            if (!client_.Connected) return new float[num];

            //读取目标变量
            var reg = tool_.AddressConvert(Address.VarGF);
            var len = sizeof(float) >> 1;
            reg[0] += (ushort)(len * index);
            num *= (ushort)len;
            var res = master_.ReadHoldingRegisters(id_, reg[0], num);

            //输出数据类型转换
            float[] ret = tool_.Ushort2T<float>(ref res, out sign);

            return ret;
        }
    }
}

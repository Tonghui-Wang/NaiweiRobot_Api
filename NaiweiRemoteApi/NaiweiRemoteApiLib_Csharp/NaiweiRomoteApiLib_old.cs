using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.Configuration;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using NModbus;
using NaiweiRobot;

namespace NaiweiRemoteApiLib
{
    public class NaiweiRomoteApiLib
    {
        private TcpClient client_;
        private IModbusMaster master_;
        private const byte id_ = 1;//从站编号，从1开始
        private string ip_ = "192.168.0.2";//通信地址
        private int port_ = 502;//通信端口
        
        //从寄存器中读取多个连续目标类型的数据
        private T[] Read<T>(Address address, ushort len = 1)
        {
            if (client_ == null || !client_.Connected) return new T[len];

            //计算待读取数据的占用的寄存器长度
            int times = Marshal.SizeOf(typeof(T)) / sizeof(ushort);
            len *= (ushort)times;

            //计算寄存器偏移地址
            var byteptr = (uint)address >> 16;//只取高16位的byte地址
            var wordptr = (ushort)(byteptr >> 1);//byte地址换算到word地址

            //读取数据，返回ushort数组
            ushort[] value = master_.ReadInputRegisters(id_, wordptr, len);

            //将ushort数组转换为目标数据类型
            return Ushorts2T<T>(ref value);
        }

        //向寄存器中写入多个连续目标类型的数据
        private void Write<T>(Address address, T[] data)
        {
            if (client_ == null || !client_.Connected) return;

            //将目标数据类型转换为ushort数组
            ushort[] value = T2Ushorts(ref data);

            //计算寄存器偏移地址
            var byteptr = (uint)address >> 16;//只取高16位的byte地址
            var wordptr = (ushort)(byteptr >> 1);//byte地址换算到word地址
            //if ((byteptr & 1) == 1) data = (ushort)(data << 8);//若byte地址为奇数，将数据内容上升到高8位

            //写入数据
            master_.WriteMultipleRegisters(id_, wordptr, value);
        }

        //批量读bool，每个bool占用8bit
        private bool[] ReadBool(Address address, ushort len = 1)
        {
            if (client_ == null || !client_.Connected) return new bool[len];

            len = (ushort)(len >> 1);
            var byteptr = (uint)address >> 16;//只取高16位的byte地址
            var wordptr = (ushort)(byteptr >> 1);//byte地址换算到word地址

            ushort[] value = master_.ReadInputRegisters(id_, wordptr, len);

            byte[] bytes = new byte[value.Length << 1];
            Buffer.BlockCopy(value, 0, bytes, 0, bytes.Length);
            int offset = 0;
            bool[] result = new bool[bytes.Length >> offset];
            for (int i = 0; i < result.Length; i++)
                result[i] = BitConverter.ToBoolean(bytes, i << offset);
            return result;
        }

        private void WriteBool(Address address, bool[] data)
        {
            if (client_ == null || !client_.Connected) return;

            int offset = 0;//源类型占用字节长度是2的offset次方
            byte[] bytes = new byte[data.Length << offset];
            Buffer.BlockCopy(data, 0, bytes, 0, bytes.Length);
            ushort[] result = new ushort[bytes.Length >> 1];
            for (int i = 0; i < result.Length; i++)
                result[i] = BitConverter.ToUInt16(bytes, i << 1);

            //计算寄存器偏移地址
            var byteptr = (uint)address >> 16;//只取高16位的byte地址
            var wordptr = (ushort)(byteptr >> 1);//byte地址换算到word地址

            //写入数据
            master_.WriteMultipleRegisters(id_, wordptr, result);
        }

        //读单个位
        private bool Read(Address address)
        {
            if (client_ == null || !client_.Connected) return false;

            var byteptr = (uint)address >> 16;//取高16位的byte地址
            var bitptr = (uint)address & 0xffff;//取低16位的bit地址
            var wordptr = (ushort)(byteptr >> 1);//byte地址换算到word地址

            var index = (ushort)(bitptr ^ (byteptr & 1) << 3);//计算待写线圈在word中的位偏移
            var val = (ushort)(1 << index);//计算第index位的参考数据

            ushort[] word = master_.ReadInputRegisters(id_, wordptr, 1);//获取寄存器当前数据
            return (word[0] & val) == val;
        }

        //写单个位
        private void Write(Address address, bool data)
        {
            if (client_ == null || !client_.Connected) return;

            var byteptr = (uint)address >> 16;//取高16位的byte地址
            var bitptr = (uint)address & 0xffff;//取低16位的bit地址
            var wordptr = (ushort)(byteptr >> 1);//byte地址换算到word地址

            var index = (ushort)(bitptr ^ (byteptr & 1) << 3);//计算待写线圈在word中的位偏移
            var val = (ushort)(1 << index);//计算第index位的参考数据

            ushort[] word = master_.ReadHoldingRegisters(id_, wordptr, 1);//获取寄存器当前数据
            //置位操作，当前寄存器数据与参考数据取或；复位操作，当前寄存器数据与参考数据的逆取与
            word[0] = (ushort)(data ? (word[0] | val) : (word[0] & ~val));//计算寄存器待写的数据

            master_.WriteSingleRegister(id_, wordptr, word[0]);//调用写寄存器实现
        }

        private T[] Ushorts2T<T>(ref ushort[] data)
        {
            byte[] bytes = new byte[data.Length << 1];
            Buffer.BlockCopy(data, 0, bytes, 0, bytes.Length);

            int offset;//目标类型占用字节长度是2的offset次方
            T[] result = new T[1];
            if (typeof(T) == typeof(short))
            {
                offset = 1;
                result = new T[bytes.Length >> offset];
                for (int i = 0; i < result.Length; i++)
                    result[i] = (T)(object)BitConverter.ToInt16(bytes, i << offset);
            }
            else if (typeof(T) == typeof(ushort))
            {
                offset = 1;
                result = new T[bytes.Length >> offset];
                for (int i = 0; i < result.Length; i++)
                    result[i] = (T)(object)BitConverter.ToUInt16(bytes, i << offset);
            }
            else if (typeof(T) == typeof(int))
            {
                offset = 2;
                result = new T[bytes.Length >> offset];
                for (int i = 0; i < result.Length; i++)
                    result[i] = (T)(object)BitConverter.ToInt32(bytes, i << offset);
            }
            else if (typeof(T) == typeof(uint))
            {
                offset = 2;
                result = new T[bytes.Length >> offset];
                for (int i = 0; i < result.Length; i++)
                    result[i] = (T)(object)BitConverter.ToUInt32(bytes, i << offset);
            }
            else if (typeof(T) == typeof(float))
            {
                offset = 2;
                result = new T[bytes.Length >> offset];
                for (int i = 0; i < result.Length; i++)
                    result[i] = (T)(object)BitConverter.ToSingle(bytes, i << offset);
            }
            else if (typeof(T) == typeof(double))
            {
                offset = 3;
                result = new T[bytes.Length >> offset];
                for (int i = 0; i < result.Length; i++)
                    result[i] = (T)(object)BitConverter.ToDouble(bytes, i << offset);
            }

            return result;
        }

        private ushort[] T2Ushorts<T>(ref T[] data)
        {
            int offset = 0;//源类型占用字节长度是2的offset次方
            if (typeof(T) == typeof(short)) offset = 1;
            else if (typeof(T) == typeof(ushort)) offset = 1;
            else if (typeof(T) == typeof(int)) offset = 2;
            else if (typeof(T) == typeof(uint)) offset = 2;
            else if (typeof(T) == typeof(float)) offset = 2;
            else if (typeof(T) == typeof(double)) offset = 3;

            byte[] bytes = new byte[data.Length << offset];
            Buffer.BlockCopy(data, 0, bytes, 0, bytes.Length);
            ushort[] result = new ushort[bytes.Length >> 1];
            for (int i = 0; i < result.Length; i++)
                result[i] = BitConverter.ToUInt16(bytes, i << 1);
            return result;
        }

        private string Ushorts2String(ushort[] data)
        {
            byte[] bytes = new byte[data.Length << 1];
            Buffer.BlockCopy(data, 0, bytes, 0, bytes.Length);
            string result = Encoding.Default.GetString(bytes);
            return result;
        }

        private ushort[] String2Ushorts(string data)
        {
            byte[] bytes = Encoding.Default.GetBytes(data.TrimEnd());
            ushort[] result = new ushort[bytes.Length >> 1];
            for (int i = 0; i < result.Length; i++)
                result[i] = BitConverter.ToUInt16(bytes, i << 1);
            return result;
        }

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

            return client_.Connected;
        }

        public bool DisConnect()
        {
            if (client_ == null) return true;
            if (!client_.Connected) return true;

            client_.Close();
            return !client_.Connected;
        }

        public bool IsConnected()
        {
            return client_.Connected;
        }

        public bool SetGlobalSpeed(short value)
        {
            Write<short>(Address.SpeedGet, new short[1] { value });
            return true;
        }

        public short GetGlobalSpeed()
        {
            var res = Read<short>(Address.SpeedGet)[0];
            return res;
        }

        public float[] GetCurJPos()
        {
            var res = Read<float>(Address.AcsPose, 9);
            return res;
        }

        public float[] GetCurWPos()
        {
            var res = Read<float>(Address.WcsPose, 9);
            return res;
        }

        public bool MotionStart()
        {
            Write(Address.AutoStop, false);
            Write(Address.AutoStart, true);
            return true;
        }

        public bool MotionPause()
        {
            Write(Address.AutoStop, false);
            Write(Address.AutoStart, false);
            Write(Address.AutoPause, true);
            return true;
        }

        public bool MotionContinue()
        {
            Write(Address.AutoPause, false);
            Write(Address.AutoStop, false);
            Write(Address.AutoStart, true);
            return true;
        }

        public bool MotionStop()
        {
            Write(Address.AutoStart, false);
            Write(Address.AutoPause, false);
            Write(Address.AutoStop, true);
            return true;
        }

        public bool SetIOValue(int IoIndex, IOType IoType, float[] dataIn)
        {
            var address = Address.Di;
            switch (IoType) 
            {
                case IOType.DI:
                    address = Address.Di;
                    break;
                case IOType.DO:
                    address= Address.Do;
                    break;
                default:
                    return false;
            }

            int i = 0;
            var data = new bool[dataIn.Length];
            foreach(float db in dataIn)
            {
                if (db == 0)
                    data[i++] = false;
                else
                    data[i++] = true;
            }

            WriteBool(address, data);
            return true;
        }

        public bool GetIOValue(int IoIndex, IOType IoType, ref float[] dataIn)
        {
            var address = Address.Di;
            switch (IoType)
            {
                case IOType.DI:
                    address = Address.Di;
                    break;
                case IOType.DO:
                    address = Address.Do;
                    break;
                default:
                    return false;
            }

            var data =ReadBool(address, (ushort)dataIn.Length);

            int i = 0;
            foreach (bool db in data)
            {
                if (db == false)
                    dataIn[i++] = 0;
                else
                    dataIn[i++] = 1;
            }

            return true;
        }

        public bool SetMultiDO(int[] IoValues)
        {
            foreach(int db in IoValues)
            {
                var num = Math.Abs(db);
                bool value = db>=0 ? true: false;
                Write(Address.Do, value);
            }

            return true;
        }

        public bool SetVarValue<T>(VarType varType, ushort varIndex, T varValue, ScopeType varScope=ScopeType.Global)
        {
            Address address = Address.M;
            switch (varType)
            {
                case VarType.EM:
                    address = Address.Em;
                    break;
                case VarType.ER:
                    address = Address.Er;
                    break;
                case VarType.M:
                    address = Address.M;
                    break;
            }

            Write<T>(address, new T[1] { varValue });
            return true;
        }

        public bool GetVarValue(VarType varType, ushort varIndex, ref string valueBack, ScopeType varScope=ScopeType.Global)
        {
            switch (varType)
            {
                case VarType.EM:
                    valueBack = Read<short>(Address.Em).ToString();
                    break;
                case VarType.ER:
                    valueBack = Read<float>(Address.Er).ToString();
                    break;
                case VarType.M:
                    valueBack = ReadBool(Address.M).ToString();
                    break;
                default:
                    valueBack= string.Empty;
                    return false;
            }

            return true;
        }

        public bool ReadInt(ushort varIndex, out int valueBack, ScopeType varScope = ScopeType.Global)
        {
            valueBack = Read<short>(Address.Em)[0];
            return true;
        }

        public bool WriteInt(ushort varIndex, short value, ScopeType varScope=ScopeType.Global)
        {
            Write<short>(Address.Em, new short[1] { value });
            return true;
        }

        public bool ReadReal(ushort varIndex, out float valueBack, ScopeType varScope=ScopeType.Global)
        {
            valueBack = Read<float>(Address.Em)[0];
            return true;
        }

        public bool WriteReal(ushort varIndex, float value, ScopeType varScope=ScopeType.Global)
        {
            Write<float>(Address.Em, new float[1] { value });
            return true;
        }

        public bool ReadStr(ushort varIndex, ScopeType varScope, ref string valueBack)
        {
            return false;
        }

        public bool WriteStr(ushort varIndex, ScopeType varScope, string value)
        {
            return false;
        }

        public bool ReadPos(short varIndex, VarType varType, ref float[] valueBack)
        {
            short data = varIndex;
            switch (varType)
            {
                case VarType.LJ:
                    data += 1000;
                    break;
                case VarType.LP:
                    data += 2000;
                    break;
                case VarType.GJ:
                    data += 3000;
                    break;
                case VarType.GP:
                    data += 4000;
                    break;
                default:
                    valueBack= new float[9];
                    return false;
            }

            Write<short>(Address.PointSel, new short[1] { data });
            valueBack = Read<float>(Address.PtSHowLink, 9);
            return true;
        }

        public bool WritePos(short varIndex, VarType varType, float[] value)
        {
            var res = Read<short>(Address.OpMode)[0];
            if (res==1) return false;

            short data = varIndex;
            switch (varType)
            {
                case VarType.LJ:
                    data += 1000;
                    break;
                case VarType.LP:
                    data += 2000;
                    break;
                case VarType.GJ:
                    data += 3000;
                    break;
                case VarType.GP:
                    data += 4000;
                    break;
                default:
                    return false;
            }

            Write<float>(Address.PtModLink, value);
            Write(Address.RemoteModify, true);
            return true;
        }

        public int E_GetErrorId()
        {
            var res = Read<short>(Address.AlarmShow)[0];
            return res;
        }

        public void E_ClearError()
        {
            Write(Address.Reset, true);
            Write(Address.Reset, false);
        }

        public bool E_SetMot(bool enable)
        {
            Write(Address.PowerSw, enable);
            return true;
        }
    }
}

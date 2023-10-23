using NModbus;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;

namespace NaiweiRobot
{
    internal class NaiweiRemoteApiTool
    {
        /// <summary>
        /// 地址转换，将机器人内存地址转为Modbus读写地址
        /// </summary>
        /// <param name="address">机器人内存地址</param>
        /// <returns>待操作寄存器起始地址及位地址</returns>
        internal ushort[] AddressConvert(Address address)
        {
            var intptr = (uint)address >> 16;//取高16位的byte地址，也即MB/MX的整数部分
            var decptr = (uint)address & 0xffff;//取低16位的bit地址，也即MX的小数部分

            var regptr = (ushort)(intptr >> 1);//byte地址换算到word地址，也即Modbus的寄存器偏移地址
            var bitptr = (ushort)(decptr ^ (intptr & 1) << 3);//计算待操作位在该寄存器中的位偏移地址

            return new ushort[2] { regptr, bitptr };
        }

        /// <summary>
        /// ushort数组转为任意数组
        /// </summary>
        /// <typeparam name="T">目标数组基本数据类型</typeparam>
        /// <param name="source">ushort源数组</param>
        /// <param name="sign">转换是否成功</param>
        /// <returns>T目标数组</returns>
        internal T[] Ushort2T<T>(ref ushort[] source, out bool sign)
        {
            //思路：ushort[]->byte[]->T[]

            sign = false;

            byte[] bytes = new byte[source.Length << 1];
            Buffer.BlockCopy(source, 0, bytes, 0, bytes.Length);
            
            T[] target;
            int offset,length;

            if(typeof(T) == typeof(bool))
            {
                offset = 0;
                length = bytes.Length >> offset;
                //if (length < 1) return new T[1];
                target = new T[length];
                for (int i = 0; i < target.Length; i++)
                    target[i] = (T)(object)BitConverter.ToBoolean(bytes, i << offset);
            }
            else if (typeof(T) == typeof(BitArray))//统一使用BitArray16的数组
            {
                offset = 1;
                length = bytes.Length << offset;
                //if (length < 1) return new T[1];
                target = new T[length];
                for (int i = 0; i < target.Length; i++)
                    target[i] = (T)(object)new BitArray(new byte[2] { bytes[2 * i], bytes[2 * i + 1] });
            }
            else if(typeof(T) == typeof(short))
            {
                offset = 1;
                length = bytes.Length >> offset;
                if (length < 1) return new T[1];
                target = new T[length];
                for (int i = 0; i < target.Length; i++)
                    target[i] = (T)(object)BitConverter.ToInt16(bytes, i << offset);
            }
            else if (typeof(T) == typeof(int))
            {
                offset = 2;
                length = bytes.Length >> offset;
                if (length < 1) return new T[1];
                target = new T[length];
                for (int i = 0; i < target.Length; i++)
                    target[i] = (T)(object)BitConverter.ToInt32(bytes, i << offset);
            }
            else if (typeof(T) == typeof(uint))
            {
                offset = 2;
                length = bytes.Length >> offset;
                if (length < 1) return new T[1];
                target = new T[length];
                for (int i = 0; i < target.Length; i++)
                    target[i] = (T)(object)BitConverter.ToUInt16(bytes, i << offset);
            }
            else if (typeof(T) == typeof(float))
            {
                offset = 2;
                length = bytes.Length >> offset;
                if (length < 1) return new T[1];
                target = new T[length];
                for (int i = 0; i < target.Length; i++)
                    target[i] = (T)(object)BitConverter.ToSingle(bytes, i << offset);
            }
            else if (typeof(T) == typeof(double))
            {
                offset = 3;
                length = bytes.Length >> offset;
                if (length < 1) return new T[1];
                target = new T[length];
                for (int i = 0; i < target.Length; i++)
                    target[i] = (T)(object)BitConverter.ToDouble(bytes, i << offset);
            }
            else
                return new T[1];

            sign = true;
            return target;
        }

        /// <summary>
        /// 任意数组转为ushort数组
        /// </summary>
        /// <typeparam name="T">源数组基本数据类型</typeparam>
        /// <param name="source">T源数组</param>
        /// <param name="sign">转换是否成功</param>
        /// <returns>ushort目标数组</returns>
        internal ushort[] T2Ushort<T>(ref T[] source, out bool sign)
        {
            //思路：T[]->byte[]->ushort[]

            //推算byte[]的大小
            int offset;
            if (typeof(T) == typeof(bool)) offset = 0;
            else if (typeof(T) == typeof(BitArray)) offset = 1;//统一使用BitArray16的数组
            else if (typeof(T) == typeof(short)) offset = 1;
            else if (typeof(T) == typeof(int)) offset = 2;
            else if (typeof(T) == typeof(uint)) offset = 2;
            else if (typeof(T) == typeof(float)) offset = 2;
            else if (typeof(T) == typeof(double)) offset = 3;
            else offset = 0;

            //若小于2个byte，无法合成至少1个ushort，说明数据有异常，退出
            sign = false;
            int length = source.Length << offset;
            if (length < 2) return new ushort[1];

            //初始化byte[]
            byte[] bytes = new byte[length];

            //将T[]的数据内容赋值给byte[]
            if (typeof(T) == typeof(BitArray))
                for (int i = 0; i < source.Length; i++)
                    ((BitArray)(object)source[i]).CopyTo(bytes, i << 1);
            else
                Buffer.BlockCopy(source, 0, bytes, 0, bytes.Length);

            //初始化ushort[]，并将byte[]的数据内容赋值给ushort[]
            ushort[] target = new ushort[bytes.Length >> 1];
            for (int i = 0; i < target.Length; i++)
                target[i] = BitConverter.ToUInt16(bytes, i << 1);

            sign = true;
            return target;
        }

        /// <summary>
        /// 位设置
        /// </summary>
        /// <param name="value">源数据</param>
        /// <param name="index">待操作位的索引</param>
        /// <param name="status">待操作位的取值</param>
        internal bool SetBit(ref ushort value, ushort index, bool status)
        {
            bool sign;
            ushort[] src = new ushort[1] { value };
            BitArray[] dst = Ushort2T<BitArray>(ref src, out sign);
            if (!sign) return sign;

            dst[0].Set(index, status);

            value = T2Ushort(ref dst, out sign)[0];
            return sign;
        }

        /// <summary>
        /// 位查询
        /// </summary>
        /// <param name="value">源数据</param>
        /// <param name="index">待操作位的索引</param>
        /// <param name="sign">操作结果</param>
        /// <returns>待操作位的值</returns>
        internal bool GetBit(ushort value, ushort index, out bool sign)
        {
            ushort[] src = new ushort[1] { value };
            BitArray[] dst = Ushort2T<BitArray>(ref src, out sign);

            var res = dst[0].Get(index);
            
            return res;
        }
    }
}

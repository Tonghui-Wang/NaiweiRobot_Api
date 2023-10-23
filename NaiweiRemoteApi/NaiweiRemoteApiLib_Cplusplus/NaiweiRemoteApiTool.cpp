#include "pch.h"
#include "NaiweiRemoteApiTool.h"
#include "NaiweiRemoteApiData.h"
#include <cstdint>

uint16_t* NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address address)
{
	auto intptr = address >> 16;//取高16位的byte地址，也即MB/MX的整数部分
	auto decptr = address & 0xffff;//取低16位的bit地址，也即MX的小数部分

	auto regptr = intptr >> 1;//byte地址换算到word地址，也即Modbus的寄存器偏移地址
	auto bitptr = decptr ^ (intptr & 1) << 3;//计算待操作位在该寄存器中的位偏移地址

	uint16_t ret[2] = { regptr, bitptr };

	return ret;
}

/// <summary>
/// 位设置
/// </summary>
/// <param name="value">源数据</param>
/// <param name="index">待操作位的索引</param>
/// <param name="status">待操作位的取值</param>
void NaiweiRemoteApiTool::SetBit(uint16_t* value, uint16_t index, bool status)
{
	*value = status ? (*value | (1 << index)) : (*value & (~(1 << index)));
}

/// <summary>
/// 位查询
/// </summary>
/// <param name="value">源数据</param>
/// <param name="index">待操作位的索引</param>
/// <returns>待操作位的值</returns>
bool NaiweiRemoteApiTool::GetBit(uint16_t value, uint16_t index)
{
	return value & (1 << index);
}

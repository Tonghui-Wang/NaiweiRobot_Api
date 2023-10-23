#include "pch.h"
#include "NaiweiRemoteApiTool.h"
#include "NaiweiRemoteApiData.h"
#include <cstdint>

uint16_t* NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address address)
{
	auto intptr = address >> 16;//ȡ��16λ��byte��ַ��Ҳ��MB/MX����������
	auto decptr = address & 0xffff;//ȡ��16λ��bit��ַ��Ҳ��MX��С������

	auto regptr = intptr >> 1;//byte��ַ���㵽word��ַ��Ҳ��Modbus�ļĴ���ƫ�Ƶ�ַ
	auto bitptr = decptr ^ (intptr & 1) << 3;//���������λ�ڸüĴ����е�λƫ�Ƶ�ַ

	uint16_t ret[2] = { regptr, bitptr };

	return ret;
}

/// <summary>
/// λ����
/// </summary>
/// <param name="value">Դ����</param>
/// <param name="index">������λ������</param>
/// <param name="status">������λ��ȡֵ</param>
void NaiweiRemoteApiTool::SetBit(uint16_t* value, uint16_t index, bool status)
{
	*value = status ? (*value | (1 << index)) : (*value & (~(1 << index)));
}

/// <summary>
/// λ��ѯ
/// </summary>
/// <param name="value">Դ����</param>
/// <param name="index">������λ������</param>
/// <returns>������λ��ֵ</returns>
bool NaiweiRemoteApiTool::GetBit(uint16_t value, uint16_t index)
{
	return value & (1 << index);
}

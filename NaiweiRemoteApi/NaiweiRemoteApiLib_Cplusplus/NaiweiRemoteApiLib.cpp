// NaiweiRemoteApiLib.cpp : Defines the exported functions for the DLL.
#include "pch.h" // use stdafx.h in Visual Studio 2017 and earlier
#include "NaiweiRemoteApiLib.h"
#include "NaiweiRemoteApiData.h"
#include "NaiweiRemoteApiTool.h"
#include "ModbusLib/modbus.h"
#include <string>
#include <vector>
#include <memory>

bool NaiweiRemoteApiLib::Connect(std::string ip, int port, bool isAutoReConnect)
{
	mbclient_ = modbus_new_tcp(ip.c_str(), port);

	auto res = modbus_set_slave(mbclient_, 1);
	if (res == -1)
	{
		modbus_free(mbclient_);
		return false;
	}

	modbus_set_response_timeout(mbclient_, 2, 0);
	res = modbus_connect(mbclient_);
	if (res == -1)
	{
		modbus_free(mbclient_);
		return false;
	}

	return true;
}

bool NaiweiRemoteApiLib::Connect(std::string& port, int baudrate, char parity, int byteBit, int stopBit)
{
	mbclient_ = modbus_new_rtu(port.c_str(), baudrate, parity, byteBit, stopBit);

	auto res = modbus_set_slave(mbclient_, 1);
	if (res == -1)
	{
		modbus_free(mbclient_);
		return false;
	}

	modbus_set_response_timeout(mbclient_, 2, 0);
	res = modbus_connect(mbclient_);
	if (res == -1)
	{
		modbus_free(mbclient_);
		return false;
	}

	return true;
}

bool NaiweiRemoteApiLib::DisConnect()
{
	modbus_flush(mbclient_);
	modbus_close(mbclient_);
	modbus_free(mbclient_);

	return true;
}

bool NaiweiRemoteApiLib::IsConnected()
{
	auto res = modbus_get_socket(mbclient_);
	return -1 == res ? false : true;
}

void NaiweiRemoteApiLib::Power(bool enable)
{	
	//若未连接，直接退出
	if (-1 == modbus_get_socket(mbclient_)) return;

	//查询当前使能状态
	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::PowerOk);
	uint16_t data;
	auto res = modbus_read_registers(mbclient_, reg[0], 1, &data);
	auto ans = NaiweiRemoteApiTool::GetBit(data, reg[1]);

	//若当前使能状态和期望使能操作已一致，直接退出
	if (enable == ans) return;

	//写入使能效果
	reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::PowerSignal);
	res = modbus_read_registers(mbclient_, reg[0], 1, &data);
	NaiweiRemoteApiTool::SetBit(&data, reg[1], true);
}

bool NaiweiRemoteApiLib::SetOpMode(NaiweiRobot::OpModeType mode)
{
	if (-1 == modbus_get_socket(mbclient_)) return false;

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::OpMode);
	auto res = modbus_write_register(mbclient_, reg[0], mode);

	return -1 == res ? false : true;
}

NaiweiRobot::OpModeType GetOpMode(bool* sign)
{
	NaiweiRobot::OpModeType ans = NaiweiRobot::OpModeType::Manual;
	if (-1 == modbus_get_socket(mbclient_)) return ans;

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::OpMode);
	auto res = modbus_read_registers(mbclient_, reg[0], 1, &ans);

	return -1 == res ? NaiweiRobot::OpModeType::Manual : ans;
}

void NaiweiRemoteApiLib::Jog(uint16_t index, int16_t direction, bool enable)
{
	if (-1 == modbus_get_socket(mbclient_)) return;

	NaiweiRobot::Address address;
	switch (direction)
	{
	case 1: address = NaiweiRobot::Address::JogPlus; break;
	case -1: address = NaiweiRobot::Address::JogMinus; break;
	default: return;
	}

	auto reg = NaiweiRemoteApiTool::AddressConvert(address);
	uint16_t data;
	auto res = modbus_read_registers(mbclient_, reg[0], 1, &data);
	if (-1 == res) return;

	NaiweiRemoteApiTool::SetBit(&data, reg[1], true);
	res = modbus_write_register(mbclient_, reg[0], data);
}

void NaiweiRemoteApiLib::Move(NaiweiRobot::VarType type, uint16_t index, bool enable)
{
	if (-1 == modbus_get_socket(mbclient_)) return;

	switch (type)
	{
	case NaiweiRobot::VarType::LJ: index += 1000; break;
	case NaiweiRobot::VarType::LP: index += 2000; break;
	case NaiweiRobot::VarType::GJ: index += 3000; break;
	case NaiweiRobot::VarType::GP: index += 4000; break;
	default: return;
	}

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::RemotePosSel);
	auto res = modbus_write_register(mbclient_, reg[0], index);

	reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::RemotePosLoc);
	uint16_t data;
	res = modbus_read_registers(mbclient_, reg[0], 1, &data);
	if (-1 == res) return;
	NaiweiRemoteApiTool::SetBit(&data, reg[1], enable);
	res = modbus_write_register(mbclient_, reg[0], data);
}

bool NaiweiRemoteApiLib::Task(uint16_t num)
{
	if (-1 == modbus_get_socket(mbclient_)) return false;

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::OpMode);
	NaiweiRobot::OpModeType data;
	auto res = modbus_read_registers(mbclient_, reg[0], 1, &data);
	if (-1 == res) return false;
	if (NaiweiRobot::OpModeType::Manual != data) return false;

	reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::TaskNumNow);
	res = modbus_write_register(mbclient_, reg[0], num);

	reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::TaskType);
	res = modbus_write_register(mbclient_, reg[0], 0);

	reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::TaskNumMod);
	res = modbus_read_registers(mbclient_, reg[0], 1, &data);
	NaiweiRemoteApiTool::SetBit(&data, reg[1], true);
	res = modbus_write_register(mbclient_, reg[0], data);

	reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::TaskSignal);
	res = modbus_read_registers(mbclient_, reg[0], 1, &data);
	NaiweiRemoteApiTool::SetBit(&data, reg[1], true);
	res = modbus_write_register(mbclient_, reg[0], data);

	reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::TaskNumExe);
	res = modbus_read_registers(mbclient_, reg[0], 1, &data);

	return num == data ? true : false;
}

bool NaiweiRemoteApiLib::SetAutoMode(uint16_t value)
{
	if (-1 == modbus_get_socket(mbclient_)) return false;

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::AutoMode);
	auto res = modbus_write_register(mbclient_, reg[0], value);

	return -1 == res ? false : true;
}

uint16_t NaiweiRemoteApiLib::GetAutoMode()
{
	uint16_t data = 0;
	if (-1 == modbus_get_socket(mbclient_)) return data;

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::AutoMode);
	auto res = modbus_read_registers(mbclient_, reg[0], 1, &data);

	return -1 == res ? 0 : data;
}

uint16_t NaiweiRemoteApiLib::GetAutoStatus()
{
	uint16_t data = 0;
	if (-1 == modbus_get_socket(mbclient_)) return data;

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::AutoStatus);	
	auto res = modbus_read_registers(mbclient_, reg[0], 1, &data);

	return -1 == res ? 0 : data;
}

bool NaiweiRemoteApiLib::MotionStart(bool enable)
{
	if (-1 == modbus_get_socket(mbclient_)) return false;

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::AutoStart);
	uint16_t data;
	auto res = modbus_read_registers(mbclient_, reg[0], 1, &data);
	if (-1 == res) return false;

	NaiweiRemoteApiTool::SetBit(&data, reg[1], enable);
	res = modbus_write_register(mbclient_, reg[0], data);

	return -1 == res ? false : true;
}

bool NaiweiRemoteApiLib::MotionPause(bool enable)
{
	if (-1 == modbus_get_socket(mbclient_)) return false;

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::AutoPause);
	uint16_t data;
	auto res = modbus_read_registers(mbclient_, reg[0], 1, &data);
	if (-1 == res) return false;

	NaiweiRemoteApiTool::SetBit(&data, reg[1], enable);
	res = modbus_write_register(mbclient_, reg[0], data);

	return -1 == res ? false : true;
}

bool NaiweiRemoteApiLib::MotionStop(bool enable)
{
	if (-1 == modbus_get_socket(mbclient_)) return false;

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::AutoStop);
	uint16_t data;
	auto res = modbus_read_registers(mbclient_, reg[0], 1, &data);
	if (-1 == res) return false;

	NaiweiRemoteApiTool::SetBit(&data, reg[1], enable);
	res = modbus_write_register(mbclient_, reg[0], data);

	return -1 == res ? false : true;
}

int NaiweiRemoteApiLib::GetError()
{
	int ans = 0;
	if (-1 == modbus_get_socket(mbclient_)) return ans;

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::AlarmCode);
	uint16_t data;
	auto res = modbus_read_registers(mbclient_, reg[0], 1, &data);
	res = NaiweiRemoteApiTool::Ushort2T<int>(&data, &ans);

	return -1 == res ? 0 : ans;
}

void NaiweiRemoteApiLib::ResetError(bool enable)
{
	if (-1 == modbus_get_socket(mbclient_)) return;

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::AlarmReset);
	uint16_t data;
	auto res = modbus_read_registers(mbclient_, reg[0], 1, &data);
	NaiweiRemoteApiTool::SetBit(&data, reg[1], enable);
	res = modbus_write_register(mbclient_, reg[0], data);
}

bool NaiweiRemoteApiLib::SetGlobalSpeed(uint16_t value)
{
	if (-1 == modbus_get_socket(mbclient_)) return false;

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::SpeedGet);
	auto res = modbus_write_register(mbclient_, reg[0], value);
	if (-1 == res) return false;

	reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::SpeedSet);
	uint16_t data;
	modbus_read_registers(mbclient_, reg[0], 1, &data);
	NaiweiRemoteApiTool::SetBit(&data, reg[1], true);
	res = modbus_write_register(mbclient_, reg[0], data);

	return -1 == res ? false : true;
}

uint16_t NaiweiRemoteApiLib::GetGlobalSpeed()
{
	uint16_t data = 0;
	if (-1 == modbus_get_socket(mbclient_)) return data;

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::SpeedGet);	
	auto res = modbus_read_registers(mbclient_, reg[0], 1, &data);

	return -1 == res ? 0 : data;
}

bool NaiweiRemoteApiLib::SetCs(NaiweiRobot::CsType cs)
{
	if (-1 == modbus_get_socket(mbclient_)) return false;

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::CsType);
	auto res = modbus_write_register(mbclient_, reg[0], cs);

	return -1 == res ? false : true;
}

NaiweiRobot::CsType NaiweiRemoteApiLib::GetCs()
{
	NaiweiRobot::CsType data = NaiweiRobot::CsType::Jcs;
	if (-1 == modbus_get_socket(mbclient_)) return data;

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::CsType);
	auto res = modbus_read_registers(mbclient_, reg[0], 1, &data);

	return -1 == res ? NaiweiRobot::CsType::Jcs : data;
}

float* NaiweiRemoteApiLib::GetCurJPos()
{
	float ans[9];
	if (-1 == modbus_get_socket(mbclient_)) return ans;

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::CurrentPosJ);
	uint16_t data[sizeof(ans) / sizeof(uint16_t)];
	auto res = modbus_read_registers(mbclient_, reg[0], sizeof(data) / sizeof(uint16_t), data);
	res = NaiweiRemoteApiTool::Ushort2T<float>(data, ans);

	return ans;

	//auto reg = std::make_unique<uint16_t[]>(2 * num);
	//auto res = modbus_read_registers(mbclient_, (int)NaiweiRobot::Address::WcsPose, num, reg.get());
	//auto pos = std::make_unique<float[]>(num);
	//for (int i = 0; i < num; i++)
	//	pos[i] = modbus_get_float_dcba(&reg[2 * i]);
	//return pos;
}

float* NaiweiRemoteApiLib::GetCurCPos()
{
	float ans[9];
	if (-1 == modbus_get_socket(mbclient_)) return ans;

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::CurrentPosC);
	uint16_t data[sizeof(ans) / sizeof(uint16_t)];
	auto res = modbus_read_registers(mbclient_, reg[0], sizeof(data) / sizeof(uint16_t), data);
	res = NaiweiRemoteApiTool::Ushort2T<float>(data, ans);

	return ans;
}

bool NaiweiRemoteApiLib::SetPos(NaiweiRobot::VarType type, uint16_t index, float* value)
{
	if (-1 == modbus_get_socket(mbclient_)) return false;

	switch (type)
	{
	case NaiweiRobot::VarType::LJ: index += 1000; break;
	case NaiweiRobot::VarType::LP: index += 2000; break;
	case NaiweiRobot::VarType::GJ: index += 3000; break;
	case NaiweiRobot::VarType::GP: index += 4000; break;
	default: return false;
	}
	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::RemotePosSel);
	auto res = modbus_write_register(mbclient_, reg[0], index);

	uint16_t data[sizeof(value) / sizeof(uint16_t)];
	res = NaiweiRemoteApiTool::T2Ushort(value, data);
	reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::RemotePosWo);
	res = modbus_write_registers(mbclient_, reg[0], sizeof(data) / sizeof(uint16_t), data);

	reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::RemotePosMod);
	res = modbus_read_registers(mbclient_, reg[0], 1, data);//借用data[0]重赋值，懒得再new中间变量了
	NaiweiRemoteApiTool::SetBit(data, reg[1], true);
	res = modbus_write_register(mbclient_, reg[0], *data);

	return true;
}

float* NaiweiRemoteApiLib::GetPos(NaiweiRobot::VarType type, uint16_t index, bool* sign)
{
	float ans[9];
	if (-1 == modbus_get_socket(mbclient_)) return ans;

	switch (type)
	{
	case NaiweiRobot::VarType::LJ: index += 1000; break;
	case NaiweiRobot::VarType::LP: index += 2000; break;
	case NaiweiRobot::VarType::GJ: index += 3000; break;
	case NaiweiRobot::VarType::GP: index += 4000; break;
	default: return false;
	}
	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::RemotePosSel);
	auto res = modbus_write_register(mbclient_, reg[0], index);

	reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::RemotePosWo);
	uint16_t data[sizeof(ans) / sizeof(uint16_t)];
	res = modbus_read_registers(mbclient_, reg[0], sizeof(data) / sizeof(uint16_t), data);
	res = NaiweiRemoteApiTool::Ushort2T<float>(data, ans);

	return ans;
}

bool NaiweiRemoteApiLib::SetIo(uint16_t index, bool* value, NaiweiRobot::IOType type)
{
	if (-1 == modbus_get_socket(mbclient_)) return false;

	if (NaiweiRobot::IOType::DO != type) return false;

	auto num = sizeof(value) / sizeof(bool);
	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::DigOut);
	reg[0] += (index >> 1);
	const int regnum = (int)(ceilf((index + num) * 0.5) - floorf(index * 0.5));
	uint16_t data[regnum];
	auto res = modbus_read_registers(mbclient_, reg[0], regnum, data);

	bool ans[sizeof(data) / sizeof(bool)];
	res = NaiweiRemoteApiTool::Ushort2T<bool>(data, ans);

	memcpy(ans[index % 2], value, sizeof(value));
	res = NaiweiRemoteApiTool::T2Ushort<bool>(ans, data);

	res = modbus_write_registers(mbclient_, reg[0], regnum, data);

	return true;
}

bool* NaiweiRemoteApiLib::GetIo(NaiweiRobot::IOType type, uint16_t index, uint16_t num, bool* sign)
{
	const int len = num;
	bool ans[len];
	if (-1 == modbus_get_socket(mbclient_)) return ans;

	NaiweiRobot::Address address;
	switch (type)
	{
	case NaiweiRobot::IOType::DI: address = NaiweiRobot::Address::DigIn; break;
	case NaiweiRobot::IOType::DO: address = NaiweiRobot::Address::DigOut; break;
	default: return ans;
	}
	auto reg = NaiweiRemoteApiTool::AddressConvert(address);
	reg[0] += (index >> 1);
	const int regnum = (int)(ceilf((index + num) * 0.5) - floorf(index * 0.5));
	uint16_t data[regnum];
	auto res = modbus_read_registers(mbclient_, reg[0], regnum, data);

	bool tmp[sizeof(data) / sizeof(bool)];
	res = NaiweiRemoteApiTool::Ushort2T<bool>(data, tmp);

	memcpy(ans[index % 2], tmp, sizeof(tmp));

	return ans;
}

bool NaiweiRemoteApiLib::SetFixDo(int* indexs, bool value = true)
{
	if (-1 == modbus_get_socket(mbclient_)) return false;
	int num = sizeof(indexs) / sizeof(bool);

	for (int i = 0; i < num; i++)
	{
		auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::DigOut);
		reg[0] += (indexs[i] >> 1);
		uint16_t data;
		auto res = modbus_read_registers(mbclient_, reg[0], 1, &data);

		bool tmp[sizeof(data) / sizeof(bool)];
		res = NaiweiRemoteApiTool::Ushort2T<bool>(&data, tmp);

		tmp[indexs[i] % 2] = value;

		res = NaiweiRemoteApiTool::T2Ushort<bool>(tmp, &data);
		res = modbus_write_register(mbclient_, reg[0], data);
	}

	return true;
}

bool* NaiweiRemoteApiLib::GetFixDo(int* index, bool* sign)
{
	const int len = num;
	bool ans[len];
	if (-1 == modbus_get_socket(mbclient_)) return ans;

	for (int i = 0; i < num; i++)
	{
		auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::DigOut);
		reg[0] += (index[i] >> 1);
		uint16_t data;
		auto res = modbus_read_registers(mbclient_, reg[0], 1, &data);

		bool tmp[sizeof(data) / sizeof(bool)];
		res = NaiweiRemoteApiTool::Ushort2T<bool>(&data, tmp);

		ans[i] = tmp[index[i] % 2];
	}

	return ans;
}

bool NaiweiRemoteApiLib::SetBool(uint16_t index, bool* value, NaiweiRobot::ScopeType scope)
{
	if (-1 == modbus_get_socket(mbclient_)) return false;

	auto num = sizeof(value) / sizeof(bool);
	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::VarGB);
	reg[0] += (index >> 1);
	const int regnum = (int)(ceilf((index + num) * 0.5) - floorf(index * 0.5));
	uint16_t data[regnum];
	auto res = modbus_read_registers(mbclient_, reg[0], regnum, data);

	bool ans[sizeof(data) / sizeof(bool)];
	res = NaiweiRemoteApiTool::Ushort2T<bool>(data, ans);

	memcpy(ans[index % 2], value, sizeof(value));
	res = NaiweiRemoteApiTool::T2Ushort<bool>(ans, data);

	res = modbus_write_registers(mbclient_, reg[0], regnum, data);

	return true;
}

bool* NaiweiRemoteApiLib::GetBool(uint16_t index, uint16_t num, bool* sign, NaiweiRobot::ScopeType scope)
{
	const int len = num;
	bool ans[len];
	if (-1 == modbus_get_socket(mbclient_)) return ans;

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::VarGB);
	reg[0] += (index >> 1);
	const int regnum = (int)(ceilf((index + num) * 0.5) - floorf(index * 0.5));
	uint16_t data[regnum];
	auto res = modbus_read_registers(mbclient_, reg[0], regnum, data);

	bool tmp[sizeof(data) / sizeof(bool)];
	res = NaiweiRemoteApiTool::Ushort2T<bool>(data, tmp);

	memcpy(ans[index % 2], tmp, sizeof(tmp));

	return ans;
}

bool NaiweiRemoteApiLib::SetInt(uint16_t index, int16_t* value, NaiweiRobot::ScopeType scope)
{
	if (-1 == modbus_get_socket(mbclient_)) return false;

	uint16_t data[sizeof(value) / sizeof(uint16_t)];
	auto res = NaiweiRemoteApiTool::T2Ushort<int16_t>(value, data);

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::VarGI);
	reg[0] += (sizeof(int16_t) >> 1) * index;//TODO:首寄存器地址处理不完整
	res = modbus_write_registers(mbclient_, reg[0], sizeof(data) / sizeof(uint16_t), data);

	return -1 == res ? false : true;
}

int16_t* NaiweiRemoteApiLib::GetInt(uint16_t index, uint16_t num, bool* sign, NaiweiRobot::ScopeType scope)
{	
	const int len = num;
	int16_t ans[len];
	if (-1 == modbus_get_socket(mbclient_)) return ans;

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::VarGI);
	reg[0] += (sizeof(int16_t) >> 1) * index;//TODO:首寄存器地址处理不完整
	uint16_t data[sizeof(ans) / sizeof(uint16_t)];
	auto res = modbus_read_registers(mbclient_, reg[0], sizeof(data) / sizeof(uint16_t), data);
	res = NaiweiRemoteApiTool::Ushort2T<int16_t>(data, ans);

	return ans;
}

bool NaiweiRemoteApiLib::SetReal(uint16_t index, float* value, NaiweiRobot::ScopeType scope)
{
	if (-1 == modbus_get_socket(mbclient_)) return false;

	uint16_t data[sizeof(value) / sizeof(uint16_t)];
	auto res = NaiweiRemoteApiTool::T2Ushort<float>(value, data);

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::VarGF);
	reg[0] += (sizeof(float) >> 1) * index;
	res = modbus_write_registers(mbclient_, reg[0], sizeof(data) / sizeof(uint16_t), data);

	return -1 == res ? false : true;
}

float* NaiweiRemoteApiLib::GetReal(uint16_t index, uint16_t num, bool* sign, NaiweiRobot::ScopeType scope)
{
	const int len = num;
	float ans[len];
	if (-1 == modbus_get_socket(mbclient_)) return ans;

	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::VarGF);
	reg[0] += (sizeof(float) >> 1) * index;
	uint16_t data[sizeof(ans) / sizeof(uint16_t)];
	auto res = modbus_read_registers(mbclient_, reg[0], sizeof(data) / sizeof(uint16_t), data);
	res = NaiweiRemoteApiTool::Ushort2T<float>(data, ans);

	return ans;
}

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
	mbclient_
	return false;
}

bool NaiweiRemoteApiLib::Power(bool enable)
{
	return false;
}

bool NaiweiRemoteApiLib::SetOpMode(NaiweiRobot::OpModeType mode)
{
	return false;
}

NaiweiRobot::OpModeType GetOpMode(bool& flag)
{
	return NaiweiRobot::OpModeType::Manual;
}

bool NaiweiRemoteApiLib::SetGlobalSpeed(int value)
{
	auto reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::SpeedGet);
	auto res = modbus_write_register(mbclient_, reg[0], (uint16_t)value);
	if (res == -1) return false;

	reg = NaiweiRemoteApiTool::AddressConvert(NaiweiRobot::Address::SpeedSet);
	uint16_t data;
	modbus_read_registers(mbclient_, reg[0], 1, &data);

	NaiweiRemoteApiTool::SetBit(data, reg[1], true);
	res = modbus_write_register(mbclient_, reg[0], data);
	if (res == -1) return false;
	return true;
}

int NaiweiRemoteApiLib::GetGlobalSpeed()
{
	uint16_t reg[1];
	auto res = modbus_read_registers(mbclient_, (int)NaiweiRobot::Address::VelSetShow, 1, reg);
	if (res == -1) return -1;
	return reg[0];
}

std::unique_ptr<float[]> NaiweiRemoteApiLib::GetCurJPos()
{
	int num = 9;
	auto reg = std::make_unique<uint16_t[]>(2 * num);
	auto res = modbus_read_registers(mbclient_, (int)NaiweiRobot::Address::WcsPose, num, reg.get());

	auto pos = std::make_unique<float[]>(num);
	for (int i = 0; i < num; i++)
		pos[i] = modbus_get_float_dcba(&reg[2 * i]);

	return pos;
}

std::unique_ptr<float[]> NaiweiRemoteApiLib::GetCurCPos()
{
	int num = 9;
	auto reg = std::make_unique<uint16_t[]>(2*num);
	auto res = modbus_read_registers(mbclient_, (int)NaiweiRobot::Address::AcsPose, num, reg.get());

	auto pos = std::make_unique<float[]>(num);
	for (int i = 0; i < num; i++)
		pos[i] = modbus_get_float_dcba(&reg[2 * i]);

	return pos;
}

bool NaiweiRemoteApiLib::MotionStart()
{
	uint16_t reg;
	int regaddress = (int)NaiweiRobot::Address::AutoStart >> 16;
	int bitaddress = (int)NaiweiRobot::Address::AutoStart & 0xffff;
	auto res = modbus_read_registers(mbclient_, regaddress, 1, &reg);
	reg |= (1 << bitaddress);
	res = modbus_write_register(mbclient_, regaddress, reg);

	res = modbus_read_registers(mbclient_, (int)NaiweiRobot::Address::TaskMode, 1, &reg);
	if (reg != 3) return false;

	res = modbus_read_registers(mbclient_, regaddress, 1, &reg);
	reg &= ~(1 << bitaddress);
	res = modbus_write_register(mbclient_, regaddress, reg);
	return true;
}


bool NaiweiRemoteApiLib::MotionPause()
{
	uint16_t reg;
	int regaddress = (int)NaiweiRobot::Address::AutoPause >> 16;
	int bitaddress = (int)NaiweiRobot::Address::AutoPause & 0xffff;
	auto res = modbus_read_registers(mbclient_, regaddress, 1, &reg);
	reg |= (1 << bitaddress);
	res = modbus_write_register(mbclient_, regaddress, reg);

	res = modbus_read_registers(mbclient_, (int)NaiweiRobot::Address::TaskMode, 1, &reg);
	if (reg != 5) return false;
	return true;
}

bool NaiweiRemoteApiLib::MotionContinue()
{
	uint16_t reg;
	int regaddress = (int)NaiweiRobot::Address::AutoStart >> 16;
	int bitaddress = (int)NaiweiRobot::Address::AutoStart & 0xffff;
	auto res = modbus_read_registers(mbclient_, regaddress, 1, &reg);
	reg |= (1 << bitaddress);
	res = modbus_write_register(mbclient_, regaddress, reg);

	res = modbus_read_registers(mbclient_, (int)NaiweiRobot::Address::TaskMode, 1, &reg);
	if (reg != 3) return false;

	res = modbus_read_registers(mbclient_, regaddress, 1, &reg);
	reg &= ~(1 << bitaddress);
	res = modbus_write_register(mbclient_, regaddress, reg);

	regaddress = (int)NaiweiRobot::Address::AutoPause >> 16;
	bitaddress = (int)NaiweiRobot::Address::AutoPause & 0xffff;
	res = modbus_read_registers(mbclient_, regaddress, 1, &reg);
	reg &= ~(1 << bitaddress);
	res = modbus_write_register(mbclient_, regaddress, reg);

	return true;
}

bool NaiweiRemoteApiLib::MotionStop()
{
	uint16_t reg;
	int regaddress = (int)NaiweiRobot::Address::AutoStop >> 16;
	int bitaddress = (int)NaiweiRobot::Address::AutoStop & 0xffff;
	auto res = modbus_read_registers(mbclient_, regaddress, 1, &reg);
	reg |= (1 << bitaddress);
	res = modbus_write_register(mbclient_, regaddress, reg);

	res = modbus_read_registers(mbclient_, (int)NaiweiRobot::Address::TaskMode, 1, &reg);
	if (reg != 5) return false;

	res = modbus_read_registers(mbclient_, regaddress, 1, &reg);
	reg &= ~(1 << bitaddress);
	res = modbus_write_register(mbclient_, regaddress, reg);

	return true;
}

//IoIndex´Ó0¿ªÊ¼
bool NaiweiRemoteApiLib::SetIOValue(int IoIndex, NaiweiRobot::IOType IoType, std::vector<double> dataIn)
{
	int regaddress = 0;
	int bitaddress = 0;

	switch (IoType)
	{
	case NaiweiRobot::IOType::DI:
		regaddress = (int)NaiweiRobot::Address::Di >> 16;
		//bitaddress = (int)NaiweiRobot::Address::Di & 0xffff;
		break;
	case NaiweiRobot::IOType::DO:
		regaddress = (int)NaiweiRobot::Address::Do >> 16;
		//bitaddress = (int)NaiweiRobot::Address::Do & 0xffff;
		break;
	//case NaiweiRobot::IOType::AI:
	//	break;
	//case NaiweiRobot::IOType::AO:
	//	break;
	//case NaiweiRobot::IOType::SIM_DI:
	//	break;
	//case NaiweiRobot::IOType::SIM_DO:
	//	break;
	//case NaiweiRobot::IOType::SIM_AI:
	//	break;
	//case NaiweiRobot::IOType::SIM_AO:
	//	break;
	default:
		return false;
		break;
	}

	regaddress += IoIndex / 2;
	bitaddress = IoIndex % 2 * 8;

	int regnum = 0;
	if (dataIn.size() % 2 == 0)
		regnum = dataIn.size() / 2 + IoIndex % 2;
	else
		regnum = ceil(dataIn.size() / 2);

	uint16_t reg;
	auto res = modbus_read_registers(mbclient_, regaddress, regnum, &reg);


	return false;
}

bool NaiweiRemoteApiLib::GetIOValue(int IoIndex, NaiweiRobot::IOType IoType, int IoNum, std::vector<double>& dataOut)
{
	return false;
}

bool NaiweiRemoteApiLib::SetMultiDO(std::vector<int> IoValues)
{
	return false;
}

bool NaiweiRemoteApiLib::SetVarValue(NaiweiRobot::VarType varType, std::string varName, std::string varValue, NaiweiRobot::ScopeType varScope)
{
	return false;
}

bool NaiweiRemoteApiLib::GetVarValue(NaiweiRobot::VarType varType, std::string varName, NaiweiRobot::ScopeType varScope, std::string& valueBack)
{
	return false;
}

bool NaiweiRemoteApiLib::SetInt(std::string varName, NaiweiRobot::ScopeType varScope, int& valueBack)
{
	return false;
}

bool NaiweiRemoteApiLib::WriteInt(std::string varName, NaiweiRobot::ScopeType varScope, int value)
{
	return false;
}

bool NaiweiRemoteApiLib::ReadReal(std::string varName, NaiweiRobot::ScopeType varScope, double& valueBack)
{
	return false;
}

bool NaiweiRemoteApiLib::WriteReal(std::string varName, NaiweiRobot::ScopeType varScope, double value)
{
	return false;
}

bool NaiweiRemoteApiLib::ReadStr(std::string varName, NaiweiRobot::ScopeType varScope, std::string& valueBack)
{
	return false;
}

bool NaiweiRemoteApiLib::WriteStr(std::string varName, NaiweiRobot::ScopeType varScope, std::string value)
{
	return false;
}

bool NaiweiRemoteApiLib::ReadPos(std::string varName, NaiweiRobot::ScopeType varScope, NaiweiRobot::ROB_POS& valueBack)
{
	return false;
}

bool NaiweiRemoteApiLib::WritePos(std::string varName, NaiweiRobot::ScopeType varScope, NaiweiRobot::ROB_POS value)
{
	return false;
}

int NaiweiRemoteApiLib::GetError()
{
	return 0;
}

void NaiweiRemoteApiLib::ResetError()
{
}


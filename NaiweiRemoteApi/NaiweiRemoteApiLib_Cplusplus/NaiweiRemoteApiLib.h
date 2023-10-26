// NaiweiRemoteApiLib.h - Contains declarations of NaiweiRemoteApiLib functions
#pragma once

#ifdef NAIWEIREMOTEAPILIB_EXPORTS
#define NAIWEIREMOTEAPILIB_API __declspec(dllexport)
#else
#define NAIWEIREMOTEAPILIB_API __declspec(dllimport)
#endif

#include "NaiweiRemoteApiData.h"
#include "ModbusLib/modbus.h"
#include <string>

class NAIWEIREMOTEAPILIB_API NaiweiRemoteApiLib
{
public:
	bool Connect(std::string ip, std::int32_t port = 502, bool isAutoReConnect = false);
	bool Connect(std::string& port, std::int32_t baudrate, char parity, std::int32_t byteBit, std::int32_t stopBit);
	bool DisConnect();
	bool IsConnected();

	void Power(bool enable = true);

	bool SetOpMode(NaiweiRobot::OpModeType mode);
	NaiweiRobot::OpModeType GetOpMode(bool* sign);

	void Jog(uint16_t index, short direction, bool enable = true);
	void Move(NaiweiRobot::VarType type, uint16_t index, bool enable);

	bool Task(uint16_t num);

	bool SetAutoMode(uint16_t value);
	uint16_t GetAutoMode();
	uint16_t GetAutoStatus();

	bool MotionStart(bool enable);
	bool MotionPause(bool enable);
	bool MotionStop(bool enable);

	int GetError();
	void ResetError(bool enable = true);

	bool SetGlobalSpeed(uint16_t value);
	uint16_t GetGlobalSpeed();

	bool SetCs(NaiweiRobot::CsType cs);
	NaiweiRobot::CsType GetCs();

	float* GetCurJPos();
	float* GetCurCPos();
	bool SetPos(NaiweiRobot::VarType type, uint16_t index, float* value);
	float* GetPos(NaiweiRobot::VarType type, uint16_t index, bool* sign);

	bool SetIo(uint16_t index, bool* value, NaiweiRobot::IOType type = NaiweiRobot::IOType::DO);
	bool* GetIo(NaiweiRobot::IOType type, uint16_t index, uint16_t num, bool* sign);
	bool SetFixDo(int* indexs, bool value = true);
	bool* GetFixDo(int* index, bool* sign);
	bool SetBool(uint16_t index, bool* value, NaiweiRobot::ScopeType scope = NaiweiRobot::ScopeType::Global);
	bool* GetBool(uint16_t index, uint16_t num, bool* sign, NaiweiRobot::ScopeType scope = NaiweiRobot::ScopeType::Global);
	bool SetInt(uint16_t index, short* value, NaiweiRobot::ScopeType scope = NaiweiRobot::ScopeType::Global);
	short* GetInt(uint16_t index, uint16_t num, bool* sign, NaiweiRobot::ScopeType scope = NaiweiRobot::ScopeType::Global);
	bool SetReal(uint16_t index, float* value, NaiweiRobot::ScopeType scope = NaiweiRobot::ScopeType::Global);
	float* GetReal(uint16_t index, uint16_t num, bool* sign, NaiweiRobot::ScopeType scope = NaiweiRobot::ScopeType::Global);

private:
	modbus_t* mbclient_;
};


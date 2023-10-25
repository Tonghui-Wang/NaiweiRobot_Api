// NaiweiRemoteApiLib.h - Contains declarations of NaiweiRemoteApiLib functions
#pragma once

#ifdef NAIWEIREMOTEAPILIB_EXPORTS
#define NAIWEIREMOTEAPILIB_API __declspec(dllexport)
#else
#define NAIWEIREMOTEAPILIB_API __declspec(dllimport)
#endif

class NAIWEIREMOTEAPILIB_API NaiweiRemoteApiLib
{
	bool Connect(std::string ip, int port = 502, bool isAutoReConnect = false);
	bool Connect(std::string& port, int baudrate, char parity, int byteBit, int stopBit);
	bool DisConnect();
	bool IsConnected();

	void Power(bool enable = true);

	bool SetOpMode(NaiweiRobot::OpModeType mode);
	NaiweiRobot::OpModeType GetOpMode(bool* sign);

	void Jog(unsigned short index, short direction, bool enable = true);
	void Move(NaiweiRobot::VarType type, unsigned short index, bool enable);

	bool Task(unsigned short num);

	bool SetAutoMode(unsigned short value);
	unsigned short GetAutoMode();
	unsigned short GetAutoStatus();

	bool MotionStart(bool enable);
	bool MotionPause(bool enable);
	bool MotionStop(bool enable);

	int GetError();
	void ResetError(bool enable = true);

	bool SetGlobalSpeed(unsigned short value);
	unsigned short GetGlobalSpeed();

	bool SetCs(NaiweiRobot::CsType cs);
	NaiweiRobot::CsType GetCs();

	float* GetCurJPos();
	float* GetCurCPos();
	bool SetPos(NaiweiRobot::VarType type, unsigned short index, float* value);
	float* GetPos(NaiweiRobot::VarType type, unsigned short index, bool* sign);

	bool SetIo(unsigned short index, bool* value, NaiweiRobot::IOType type = NaiweiRobot::IOType::DO);
	bool* GetIo(NaiweiRobot::IOType type, unsigned short index, unsigned short num, bool* sign);
	bool SetFixDo(int* indexs, bool value = true);
	bool* GetFixDo(int* index, bool* sign);
	bool SetBool(unsigned short index, bool* value, NaiweiRobot::ScopeType scope = NaiweiRobot::ScopeType::Global);
	bool* GetBool(unsigned short index, unsigned short num, bool* sign, NaiweiRobot::ScopeType scope = NaiweiRobot::ScopeType::Global);
	bool SetInt(unsigned short index, short* value, NaiweiRobot::ScopeType scope = NaiweiRobot::ScopeType::Global);
	short* GetInt(unsigned short index, unsigned short num, bool* sign, NaiweiRobot::ScopeType scope = NaiweiRobot::ScopeType::Global);
	bool SetReal(unsigned short index, float* value, NaiweiRobot::ScopeType scope = NaiweiRobot::ScopeType::Global);
	float* GetReal(unsigned short index, unsigned short num, bool* sign, NaiweiRobot::ScopeType scope = NaiweiRobot::ScopeType::Global);

//private:
	modbus_t* mbclient_;
};


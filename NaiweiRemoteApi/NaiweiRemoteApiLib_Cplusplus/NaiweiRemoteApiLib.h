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
	NaiweiRobot::OpModeType GetOpMode(bool& sign);

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

	std::unique_ptr<float[]> GetCurJPos();
	std::unique_ptr<float[]> GetCurCPos();
	bool SetPos(NaiweiRobot::VarType type, unsigned short index, std::vector<float> value);
	std::unique_ptr<float[]> GetPos(NaiweiRobot::VarType type, unsigned short index, bool& sign);

	bool SetIo(unsigned short index, std::vector<bool> value, NaiweiRobot::IOType type = NaiweiRobot::IOType::DO);
	std::unique_ptr<bool[]> GetIo(NaiweiRobot::IOType type, unsigned short index, unsigned short num, bool& sign);
	bool SetFixDo(std::vector<int> indexs, bool value = true);
	std::unique_ptr<bool[]> GetFixDo(std::vector<int> index, bool& sign);
	bool SetBool(unsigned short index, std::vector<bool> value, NaiweiRobot::ScopeType scope = NaiweiRobot::ScopeType::Global);
	std::unique_ptr<bool[]> GetBool(unsigned short index, unsigned short num, bool& sign, NaiweiRobot::ScopeType scope = NaiweiRobot::ScopeType::Global);
	bool SetInt(unsigned short index, std::vector<short> value, NaiweiRobot::ScopeType scope = NaiweiRobot::ScopeType::Global);
	std::unique_ptr<short[]> GetInt(unsigned short index, unsigned short num, bool& sign, NaiweiRobot::ScopeType scope = NaiweiRobot::ScopeType::Global);
	bool SetReal(unsigned short index, std::vector<float> value, NaiweiRobot::ScopeType scope = NaiweiRobot::ScopeType::Global);
	std::unique_ptr<float[]> GetReal(unsigned short index, unsigned short num, bool& sign, NaiweiRobot::ScopeType scope = NaiweiRobot::ScopeType::Global);

	bool SetIOValue(int IoIndex, NaiweiRobot::IOType IoType, std::vector<double> dataIn);
	bool GetIOValue(int IoIndex, NaiweiRobot::IOType IoType, int IoNum, std::vector<double>& dataOut);
	bool SetMultiDO(std::vector<int> IoValues);
	bool SetVarValue(NaiweiRobot::VarType varType, std::string varName, std::string varValue, NaiweiRobot::ScopeType varScope);
	bool GetVarValue(NaiweiRobot::VarType varType, std::string varName, NaiweiRobot::ScopeType varScope, std::string & valueBack);
	bool SetInt(std::string varName, NaiweiRobot::ScopeType varScope, int & valueBack);
	bool WriteInt(std::string varName, NaiweiRobot::ScopeType varScope, int value);
	bool ReadReal(std::string varName, NaiweiRobot::ScopeType varScope, double & valueBack);
	bool WriteReal(std::string varName, NaiweiRobot::ScopeType varScope, double value);
	bool ReadStr(std::string varName, NaiweiRobot::ScopeType varScope, std::string& valueBack);
	bool WriteStr(std::string varName, NaiweiRobot::ScopeType varScope, std::string value);
	bool ReadPos(std::string varName, NaiweiRobot::ScopeType varScope, NaiweiRobot::ROB_POS& valueBack);
	bool WritePos(std::string varName, NaiweiRobot::ScopeType varScope, NaiweiRobot::ROB_POS value);

//private:
	modbus_t* mbclient_;
};


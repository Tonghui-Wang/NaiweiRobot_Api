#pragma once

#ifdef NAIWEIREMOTEAPITOOL_EXPORTS
#define NAIWEIREMOTEAPITOOL_API __declspec(dllexport)
#else
#define NAIWEIREMOTEAPITOOL_API __declspec(dllimport)
#endif

class NAIWEIREMOTEAPITOOL_API NaiweiRemoteApiTool
{
private:
	uint16_t* AddressConvert(NaiweiRobot::Address address);

	template<typename T>
	T* Ushort2T(uint16_t* source, bool* sign)
	{
		sign = false;

		T* ret;
		memcpy(ret, source, sizeof(source));

		return ret;
	}

	template<typename T>
	uint16_t* T2Ushort(T* source, bool* sign)
	{
		sign = false;

		uint16_t* ret;
		memcpy(ret, source, sizeof(source));

		return ret;
	}

	void SetBit(uint16_t* value, uint16_t index, bool status);
	bool GetBit(uint16_t value, uint16_t index);
};

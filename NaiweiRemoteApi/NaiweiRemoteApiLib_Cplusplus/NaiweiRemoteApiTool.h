#pragma once

#ifdef NAIWEIREMOTEAPITOOL_EXPORTS
#define NAIWEIREMOTEAPITOOL_API __declspec(dllexport)
#else
#define NAIWEIREMOTEAPITOOL_API __declspec(dllimport)
#endif

class NAIWEIREMOTEAPITOOL_API NaiweiRemoteApiTool
{
public:
	static uint16_t* AddressConvert(NaiweiRobot::Address address);

	template<typename T>
	static int Ushort2T(uint16_t* source, T* dest)
	{
		//T* dest = (T*)malloc(sizeof(source));
		memcpy(dest, source, sizeof(source));

		return 1;
	}

	template<typename T>
	static int T2Ushort(T* source, uint16_t* dest)
	{
		//uint16_t* dest = (T*)malloc(sizeof(source));
		memcpy(dest, source, sizeof(source));

		return 1;
	}

	static void SetBit(uint16_t* value, uint16_t index, bool status);
	static bool GetBit(uint16_t value, uint16_t index);
};

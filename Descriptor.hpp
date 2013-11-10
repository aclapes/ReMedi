#pragma once

template<typename T>
class Descriptor
{
public:
 
	Descriptor(T descriptor) : m_Descriptor(descriptor) { }
	~Descriptor(void) {}

private:
	T m_Descriptor;
};


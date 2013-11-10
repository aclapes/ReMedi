#pragma once

#include "Reader.h"
#include "DepthFrame.h"

#include "InteractiveRegisterer.h"
#include "BackgroundSubtractor.h"
#include "Monitorizer.h"
#include "Cloudject.hpp"


class Remedi
{
public:

	// Constructors
	Remedi();
	~Remedi();

	// Public methods
	void Run();

private:

	//// Private members
	//ImageFileReader					m_imageFileReader;
	//InteractiveCloudRegisterer		m_InteractiveCloudRegisterer;
	//BackgroundSubtractor			m_BackgroundSubtractor;
	//Monitorizer						m_Monitorizer;
};
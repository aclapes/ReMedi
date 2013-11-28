#pragma once

#include "Reader.h"
#include "InteractiveRegisterer.h"
#include "BackgroundSubtractor.h"
#include "TableModeler.h"


class Remedi
{
public:

	// Constructors
	Remedi();
	~Remedi();

	// Public methods
	void Run();

private:
	void waitForBackgroundSubtraction(Reader& reader, BackgroundSubtractor& bs);
	void interactWithRegisterer(InteractiveRegisterer& registerer, Reader& reader); // ugly: to pass the reader
	void modelTables(TableModeler& tableModeler, InteractiveRegisterer& registerer, Reader& reader);
};
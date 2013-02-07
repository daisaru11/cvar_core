//
//  MapCellStatus.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#ifndef __cvar_core__MapCellStatus__
#define __cvar_core__MapCellStatus__

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>

class MapCellStatus {
public:
	enum CellStatus {
		CSTATUS_NONE = 0,
		CSTATUS_ONBORDER = 1,
		CSTATUS_INNERBORDER = 2,
	};

	enum MapStatus {
		CMSTATUS_NONE = 0,
		CMSTATUS_MAPPING = 1,
		CMSTATUS_MAPPED = 2
	};

	CellStatus status;
	MapStatus mapped;
	int col;
	int row;

	MapCellStatus();

protected:
private:
};

#endif /* defined(__cvar_core__MapCellStatus__) */

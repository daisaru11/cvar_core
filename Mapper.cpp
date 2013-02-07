//
//  Mapper.cpp
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//

#include "Mapper.h"

Mapper::Mapper(const cv::Size _nrm_map_size, const cv::Size _img_size, 
const std::vector<double> _scales,
const std::vector<size_t> _max_kp_per_cell)
:nrm_map_size(_nrm_map_size), img_size(_img_size),
nrm_cell_size(60,60),
scales(_scales),
img_center(_img_size.width/2,_img_size.height/2),
cell_rows(_nrm_map_size.height/nrm_cell_size.height),
cell_cols(_nrm_map_size.width/nrm_cell_size.width),
cells_st(cell_rows, std::vector<MapCellStatus>(cell_cols)),
#ifdef CVAR_PC_MOCK
kp_detector(5,true),
#else
kp_detector(20,true),
#endif
kp_id_cnt(0),
max_kp_per_cell(_max_kp_per_cell),
img_store(2),
gray_store(2)
{
	for (size_t i=0; i<scales.size(); ++i)
	{
		const double s = scales[i];
		const cv::Size map_size(
			static_cast<int>(nrm_map_size.width*s), 
			static_cast<int>(nrm_map_size.height*s) );
		const cv::Size cell_size(
			static_cast<int>(nrm_cell_size.width*s), 
			static_cast<int>(nrm_cell_size.height*s) );
		maps.push_back(Map(map_size, cell_size, cell_rows, cell_cols));
	}

    for (int j=0; j<cell_rows; ++j) {
        for (int i=0; i<cell_cols; ++i) {
			MapCellStatus& st = cells_st[j][i];
			st.col = i;
			st.row = j;
        }
    }
}

Mapper::~Mapper()
{
}

bool Mapper::detectResultCell()
{
	Map& map = maps[0];
	cv::Size map_size = map.size;
	cv::Point map_center = map.center;
	cv::Size cell_size = map.cell_size;
	CylindricalProjector& projector = map.projector;
	
    const cv::Point dfs[4] = {
        cv::Point(1,0),
        cv::Point(1,0),
        cv::Point(0,1),
        cv::Point(0,1)
    };
    
    cv::Point ranges[4][2] = {};  //start point and end point
    ranges[0][0] = cv::Point(0,0);               ranges[0][1] = cv::Point(img_size.width-1,0);
    ranges[1][0] = cv::Point(0,img_size.height-1); ranges[1][1] = cv::Point(img_size.width-1,img_size.height-1);
    ranges[2][0] = cv::Point(0,0);               ranges[2][1] = cv::Point(0,img_size.height-1);
    ranges[3][0] = cv::Point(img_size.width-1,0);  ranges[3][1] = cv::Point(img_size.width-1,img_size.height-1);
    
    double tl_uf = std::numeric_limits<double>::max();
    double tl_vf = std::numeric_limits<double>::max();
    double br_uf = -std::numeric_limits<double>::max();
    double br_vf = -std::numeric_limits<double>::max();
    
    double u, v;
    for (int k=0; k<4; ++k)
    {
        cv::Point df = dfs[k];
        for (cv::Point p=ranges[k][0], endp=ranges[k][1]; p.x<=endp.x&&p.y<=endp.y; p+=df) {
            projector.mapForward(static_cast<double>(p.x), static_cast<double>(p.y), u, v);
            
            u+=map_center.x; // convert to mat coordinate
            v+=map_center.y;
            
            int i = static_cast<int>(u) / cell_size.width;
            int j = static_cast<int>(v) / cell_size.height;
            
            if ( 0 <= j && j < cell_rows && 0 <= i && i < cell_cols )
            {
                cells_st[j][i].status = MapCellStatus::CSTATUS_ONBORDER;
            }
            
        }
    }

	projector.mapForward(static_cast<double>(img_center.x), static_cast<double>(img_center.y), u, v);
	u+=map_center.x; // convert to mat coordinate
	v+=map_center.y;
	const int center_cell_i = static_cast<int>(u) / cell_size.width;
	const int center_cell_j = static_cast<int>(v) / cell_size.height;
	if ( 0 <= center_cell_j && center_cell_j < cell_rows 
		&& 0 <= center_cell_i && center_cell_i < cell_cols )
	{
	} else {
		std::cerr << "image center is out of map" << std::endl;
		return false;
	}

	detectInnerCell(center_cell_i, center_cell_j);

    for (size_t j=0; j<cell_rows; ++j) {
        for (size_t i=0; i<cell_cols; ++i) {
			if (cells_st[j][i].status==MapCellStatus::CSTATUS_INNERBORDER && cells_st[j][i].mapped==MapCellStatus::CMSTATUS_NONE)
				cells_st[j][i].mapped = MapCellStatus::CMSTATUS_MAPPING;
		}
	}

	/* *
    std::cout << "status" << std::endl;
    for (size_t j=0; j<cell_rows; ++j) {
        for (size_t i=0; i<cell_cols; ++i) {
            std::cout << cells_st[j][i].status << ",";
        }
        std::cout << std::endl;
    }
	/* */

	return true;
}

void Mapper::fillInnerRec(const int i, const int j)
{
	cells_st[j][i].status = MapCellStatus::CSTATUS_INNERBORDER;

	if (i-1>=0 && cells_st[j][i-1].status==MapCellStatus::CSTATUS_NONE) 
		fillInnerRec(i-1,j);
	if (i+1<cell_cols && cells_st[j][i+1].status==MapCellStatus::CSTATUS_NONE) 
		fillInnerRec(i+1,j);
	if (j-1>=0 && cells_st[j-1][i].status==MapCellStatus::CSTATUS_NONE) 
		fillInnerRec(i,j-1);
	if (j+1<cell_rows && cells_st[j+1][i].status==MapCellStatus::CSTATUS_NONE)
		fillInnerRec(i,j+1);

}

bool Mapper::detectInnerCell(const int center_cell_i, const int center_cell_j)
{
	if ( 0 <= center_cell_j && center_cell_j < cell_rows 
		&& 0 <= center_cell_i && center_cell_i < cell_cols )
	{
		if (cells_st[center_cell_j][center_cell_i].status == MapCellStatus::CSTATUS_NONE)
		{
			fillInnerRec(center_cell_i, center_cell_j);
			return true;
		}
	}
	return false;
}

void Mapper::resetCellStatus()
{
    for (size_t j=0; j<cell_rows; ++j) {
        for (size_t i=0; i<cell_cols; ++i) {
            cells_st[j][i].status = MapCellStatus::CSTATUS_NONE;
        }
    }
}

void Mapper::mapCellRoi(const cv::Mat& img, const int i, const int j) {

	Map& nmap = maps[0]; //nrm_map
	cv::Point nrm_map_center = nmap.center;
	CylindricalProjector& projector = nmap.projector;
	
    Mat xmap, ymap;
    xmap.create(nrm_cell_size.height, nrm_cell_size.width, CV_32F);
    ymap.create(nrm_cell_size.height, nrm_cell_size.width, CV_32F);
	
	//time_t t0,t1;
	//t0 = clock();
	double x, y;
	int us = nrm_cell_size.width*i - nrm_map_center.x;
	int ue = us + nrm_cell_size.width;
	int vs = nrm_cell_size.height*j - nrm_map_center.y;
	int ve = vs + nrm_cell_size.height;
	float* xmapp = (float*)xmap.data; //!TODO check 
	float* ymapp = (float*)ymap.data;
	for (int v = vs; v < ve ; ++v)
	{
		for (int u = us; u < ue; ++u)
		{
			projector.mapBackward(static_cast<double>(u), static_cast<double>(v), x, y);
			//xmap.at<float>(v - vs, u - us) = static_cast<float>(x);
			//ymap.at<float>(v - vs, u - us) = static_cast<float>(y);
			*xmapp = static_cast<float>(x);
			*ymapp = static_cast<float>(y);
			xmapp++;
			ymapp++;
        }
    }

    cv::Mat tmp;
    cv::remap(img, tmp, xmap, ymap, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
	//t1 = clock();
	//std::cout << "remap : " << (double)(t1-t0)/CLOCKS_PER_SEC << std::endl;
	
	// copy tmp to map roi
    cv::Mat nmap_roi(nmap.img, nmap.cells[j][i].roi);
	cv::Mat nmap_gray_roi(nmap.gray, nmap.cells[j][i].roi);
	copyCellRoi(tmp, nmap_roi);
	cv::cvtColor(nmap_roi, nmap_gray_roi, CV_BGR2GRAY);

	// resize and copy the map cell to low-resolution maps
	for (size_t k=1; k<maps.size(); ++k)
	{
		Map& m = maps[k];
		cv::Mat low_map_roi(m.img, m.cells[j][i].roi);
		cv::Mat low_map_gray_roi(m.gray, m.cells[j][i].roi);
		/* */
		cv::resize(nmap_gray_roi, low_map_gray_roi, m.cell_size);
		/* */
		cv::resize(nmap_roi, low_map_roi, m.cell_size);
		/* */
	}
}

void Mapper::copyCellRoi(const cv::Mat& src, cv::Mat& dst_roi)
{
	const uchar* sptr = src.data;
	uchar* dptr = dst_roi.data;
	// to handle the copying 1xn matrix => nx1 std vector.
	Size sz = src.size();
	size_t len = sz.width*src.elemSize();

	for( ; sz.height--; sptr += src.step, dptr += dst_roi.step )
		memcpy( dptr, sptr, len );
}

void Mapper::updateCellStatus()
{
	resetCellStatus();
	detectResultCell();
}

void Mapper::updateMap()
{
    clock_t t0,t1;
	const cv::Mat img = getFrameImage(0);
    for (int j=0; j<cell_rows; ++j) {
        for (int i=0; i<cell_cols; ++i) {
			// maping new cells
			if (cells_st[j][i].mapped == MapCellStatus::CMSTATUS_MAPPING) 
			{
				//t0 = clock();
				mapCellRoi(img, i, j);
				//t1 = clock();
				//std::cout << "map roi: " << (double)(t1-t0)/CLOCKS_PER_SEC << std::endl;
				detectKeypointsInCell(i, j);
				cells_st[j][i].mapped = MapCellStatus::CMSTATUS_MAPPED;
			}

		}
	}
}

void Mapper::trackPoints(const int search_range, std::vector<Keypoint>& keypoints, int& drop_count, const int using_kp_max)
{
	KeypointMatcher matcher(search_range, 0.94, false);
	//const cv::Mat img = getFrameImage(si);
	const cv::Mat img = getFrameGray(0);

	drop_count = 0;
	matchPointsOnCells(img, maps[0], matcher, keypoints, drop_count, using_kp_max);
}

void Mapper::trackPointsRoughly(const int search_range, std::vector<Keypoint>& keypoints, int& drop_count, const int using_kp_max)
{
	// at first, search keypoints on the low-resolution map
	KeypointMatcher matcher(search_range, 0.90, false);
	const int si = 1; // choose scale

	//const cv::Mat img = getFrameImage(si);
	const cv::Mat img = getFrameGray(si);

	drop_count = 0;
	matchPointsOnCells(img, maps[si], matcher, keypoints, drop_count, using_kp_max);
}

void Mapper::matchPointsOnCells(const cv::Mat& img, Map& map, const KeypointMatcher& matcher, std::vector<Keypoint>& keypoints, int& drop_count, const int using_kp_max)
{
	if (using_kp_max>0) {
		for (int kp_idx=0; ;++kp_idx)
		{
			bool kp_remaining = false;
			for (size_t j=0; j<cell_rows; ++j) {
				for (size_t i=0; i<cell_cols; ++i) {
					MapCell& cell = map.cells[j][i];
					MapCellStatus& st = cells_st[j][i];

					// cells to track
					if (st.status==MapCellStatus::CSTATUS_INNERBORDER && st.mapped==MapCellStatus::CMSTATUS_MAPPED) {
						if ( cell.keypoints.size() > kp_idx ) {
							kp_remaining = true;
							Keypoint& kp = cell.keypoints[kp_idx];
							bool found = matchKeypoint(img, map, matcher, kp);
							if (found) {
								keypoints.push_back(kp);
								if (keypoints.size()>=using_kp_max) return;
							} else {
								++drop_count;
							}
						}
					}
				}
			}
			if (!kp_remaining) return;
		}
	} else {
		for (size_t j=0; j<cell_rows; ++j) {
			for (size_t i=0; i<cell_cols; ++i) {
				MapCell& cell = map.cells[j][i];
				MapCellStatus& st = cells_st[j][i];

				// cells to track
				if (st.status==MapCellStatus::CSTATUS_INNERBORDER && st.mapped==MapCellStatus::CMSTATUS_MAPPED) {
					for (std::vector<Keypoint>::iterator kp=cell.keypoints.begin(); kp!=cell.keypoints.end(); ++kp)
					{
						bool found = matchKeypoint(img, map, matcher, (*kp));
						if (found) {
							keypoints.push_back(*kp);
						} else {
							++drop_count;
						}
					}
				}
			}
		}
	}
}

bool Mapper::matchKeypoint(const cv::Mat& img, Map& map, const KeypointMatcher& matcher, Keypoint& kp)
{
	CylindricalProjector& projector = map.projector;
	
	const cv::Point2d& map_point = kp.map_point;
	cv::Point2d& img_point = kp.img_point;

	//time_t t0,t1;
	//t0 = clock();

	// estimated keypoint on image
	const double dx = static_cast<double>(map.center.x), dy = static_cast<double>(map.center.y);
	projector.mapBackward(map_point.x-dx, map_point.y-dy, img_point.x, img_point.y);

	bool found;

	// warp patch
	/* */
	found = matchWarpedPatch(img, map, matcher, kp);
	/* *
	// direct match
	if(img.channels() == 3)
		found = matcher.matchPoint(map.img, map_point, img, img_point);
	else
		found = matcher.matchPoint(map.gray, map_point, img, img_point);
	/* */

	//t1 = clock();
	//std::cout << "matcher remap: " << (double)(t1-t0)/CLOCKS_PER_SEC << std::endl;


	return found;
}

bool Mapper::matchWarpedPatch(const cv::Mat& img, Map& map, const KeypointMatcher& matcher, Keypoint& kp)
{
	const double dx = static_cast<double>(map.center.x), dy = static_cast<double>(map.center.y);
	CylindricalProjector& projector = map.projector;
	
	const cv::Point2d& map_point = kp.map_point;
	cv::Point2d& img_point = kp.img_point;
	// img_point whose position is needed to be estimated roughly
	// we search the point matched with keypoint on the map around the estimated image point
	
	// warped keypoint on image
	double warped_map_point_x, warped_map_point_y;
	projector.mapBackward(map_point.x-dx, map_point.y-dy, warped_map_point_x, warped_map_point_y);
	
#ifdef CVAR_IOS
	const int patch_half = 3;
#elif defined CVAR_PC_MOCK
	const int patch_half = 3;
#else
	const int patch_half = 3;
#endif
	const int patch_width = patch_half*2+1;
    Mat xmap, ymap;
    xmap.create(patch_width, patch_width, CV_32F);
    ymap.create(patch_width, patch_width, CV_32F);
    double u, v;
	const int xs = static_cast<int>(warped_map_point_x) - patch_half;
	const int xe = xs + patch_width;
	const int ys = static_cast<int>(warped_map_point_y) - patch_half;
	const int ye = ys + patch_width;
	float* xmapp = (float*) xmap.data; // ! TODO check data is continous?
	float* ymapp = (float*) ymap.data;
	for (int y = ys; y < ye; ++y)
	{
		for (int x = xs; x < xe ; ++x)
		{
            projector.mapForward(static_cast<double>(x), static_cast<double>(y), u, v);
			u += dx; v += dy;
            //xmap.at<float>(y - ys, x - xs) = static_cast<float>(u);
            //ymap.at<float>(y - ys, x - xs) = static_cast<float>(v);
			*xmapp = static_cast<float>(u);
            *ymapp = static_cast<float>(v);
			++xmapp; ++ymapp;
        }
    }

	// remap map to img world
    cv::Mat tmp;
	if(img.channels() == 3)
	{
		cv::remap(map.img, tmp, xmap, ymap, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
	}
	else
	{
		cv::remap(map.gray, tmp, xmap, ymap, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
	}

	bool found = matcher.matchPatch(tmp, img, img_point);
	img_point.x += patch_half; // add patch window size to get center
	img_point.y += patch_half;

	return found;
}



class CompareCvKeyPointDesc {
public:
	bool operator()(const cv::KeyPoint& first, const cv::KeyPoint& second)
	{
		return first.response > second.response;
	}
};

void Mapper::detectKeypointsInCell(const int i, const int j)
{
	for (size_t s=0; s<maps.size(); ++s)
	{
		detectKeypointsInCell(
			maps[s],
			max_kp_per_cell[s],
			i, j);
	}
}

void Mapper::detectKeypointsInCell(Map& map, size_t max_kp_count, const int i, const int j)
{
	MapCell& cell = map.cells[j][i];
	
    //const cv::Mat roi(map.img, cell.roi);
	//cv::Mat roi_gray;
	//cv::cvtColor(roi, roi_gray, CV_BGR2GRAY);
	const cv::Mat roi_gray(map.gray, cell.roi);


	std::vector<cv::KeyPoint> cv_kps;
	kp_detector.detect(roi_gray, cv_kps);
	
	std::sort(cv_kps.begin(), cv_kps.end(), CompareCvKeyPointDesc());

	const size_t klen = max_kp_count<cv_kps.size() ? max_kp_count : cv_kps.size();
	cell.keypoints.resize(klen);

	const double dx = map.cell_size.width*i;
	const double dy = map.cell_size.height*j;
	for (size_t k=0; k<klen; ++k)
	{
		cell.keypoints[k].map_point.x = dx + cv_kps[k].pt.x;
		cell.keypoints[k].map_point.y = dy + cv_kps[k].pt.y;
		cell.keypoints[k].id = getNewKeypointId(); // 
	}

	//cv::imshow("kp", roi_gray);
	//char key = (char)cv::waitKey(0);
	
	// copy keypoints to low-resolution cells
	/*
	for (size_t l=1; l<maps.size(); ++l)
	{
		MapCell& cell = maps[l].cells[j][i];
		cell.keypoints.resize(klen);
		double scale = scales[l];

		for (size_t k=0; k<klen; ++k)
		{
			cell.keypoints[k].map_point.x = ncell.keypoints[k].map_point.x * scale;
			cell.keypoints[k].map_point.y = ncell.keypoints[k].map_point.y * scale;
			cell.keypoints[k].id = ncell.keypoints[k].id;
		}
	}
	*/
}

void Mapper::backProjectPointToImage(const cv::Point2d& src, cv::Point2d& dst)
{
	Map& map = maps[0];
	const cv::Point map_center = map.center;
	const double dx = static_cast<double>(map_center.x), dy = static_cast<double>(map_center.y);
	CylindricalProjector& projector = map.projector;

	projector.mapBackward(src.x-dx, src.y-dy, dst.x, dst.y);
}

void Mapper::projectPointToMap(const cv::Point2d& src, cv::Point2d& dst, const size_t scale_idx)
{
	Map& map = maps[scale_idx];
	const cv::Point map_center = map.center;
	const double dx = static_cast<double>(map_center.x), dy = static_cast<double>(map_center.y);
	CylindricalProjector& projector = map.projector;

	projector.mapForward(src.x, src.y, dst.x, dst.y);
	dst.x += dx;
	dst.y += dy;
}

void Mapper::drawDebugMap(cv::Mat& output, const size_t scale_idx, const cv::Size& frame_size)
{
	const double scale = scales[scale_idx];
	const Map& map = maps[scale_idx];

	const int w_margin = output.cols / 48;
	const double draw_width = output.cols - 2*w_margin;
	const double draw_scale = draw_width / map.size.width;
	const double draw_height = map.size.height * draw_scale;
	const double draw_cell_height = map.cell_size.height * draw_scale;
	const double draw_cell_width = map.cell_size.width * draw_scale;

	double dx,dy;
	
	dy = output.rows-w_margin-draw_height;
	for (size_t j=0; j<=map.cell_rows; ++j)
	{
		dx = w_margin;
		for (size_t i=0; i<=map.cell_cols; ++i)
		{
			//cv::Rect out_roi_rect(dx,dy,draw_cell_width,draw_cell_height);
			//cv::Mat out_roi(output, out_roi_rect);
			//cv::Mat cell_roi(map.img,map.cells[j][i].roi);
			//cv::resize(cell_roi, out_roi, cv::Size(draw_cell_width,draw_cell_height))
			dx += draw_cell_width;
		}
		dy += draw_cell_height;
	}
	
	dx = w_margin;
	dy = output.rows-w_margin-draw_height;
	cv::Rect out_roi_rect((int)dx,(int)dy,(int)draw_width,(int)draw_height);
	cv::Mat out_roi(output, out_roi_rect);
	cv::resize(map.img, out_roi, cv::Size((int)draw_width,(int)draw_height));
	
	// draw grid
	dy = output.rows-w_margin-draw_height;
	for (size_t j=0; j<=map.cell_rows; ++j)
	{
		cv::Point p0(w_margin,(int)dy);
		cv::Point p1(output.cols-w_margin,(int)dy);
		cv::line(output, p0, p1, cv::Scalar(0,100,100), 1);
		dy += draw_cell_height;
	}
	dx = w_margin;
	for (size_t i=0; i<=map.cell_cols; ++i)
	{
		cv::Point p0((int)dx, output.rows-w_margin-(int)draw_height);
		cv::Point p1((int)dx, output.rows-w_margin);
		cv::line(output, p0, p1, cv::Scalar(0,100,100), 1);
		dx += draw_cell_width;
	}

	// draw cursor
	cv::Point2d lt, lb, rt, rb;
	projectPointToMap( cv::Point2d(0,0), lt, 1);
	projectPointToMap( cv::Point2d(0,frame_size.height), lb, 1);
	projectPointToMap( cv::Point2d(frame_size.width,0), rt, 1);
	projectPointToMap( cv::Point2d(frame_size.width,frame_size.height), rb, 1);
	lt *= draw_scale; lb *= draw_scale;
	rt *= draw_scale; rb *= draw_scale;
	cv::line(out_roi, lt, lb, cv::Scalar(0, 255, 0), 1);
	cv::line(out_roi, lb, rb, cv::Scalar(0, 255, 0), 1);
	cv::line(out_roi, rb, rt, cv::Scalar(0, 255, 0), 1);
	cv::line(out_roi, rt, lt, cv::Scalar(0, 255, 0), 1);
}

/*
void Mapper::project(const cv::Mat& img, const cv::Mat& R_)
{
	cv::Mat R= R_.t();
	projector.scale = 1048;
    projector.setCameraParams(K, R);
	
    Point dst_tl, dst_br;
	Size src_size = img.size();
	{
		float tl_uf = std::numeric_limits<float>::max();
		float tl_vf = std::numeric_limits<float>::max();
		float br_uf = -std::numeric_limits<float>::max();
		float br_vf = -std::numeric_limits<float>::max();

		float u, v;
		for (int y = 0; y < src_size.height; ++y)
		{
			for (int x = 0; x < src_size.width; ++x)
			{
				//projector.mapForward(static_cast<float>(x-img_center.x), static_cast<float>(y-img_center.y), u, v);
				projector.mapForward(static_cast<float>(x), static_cast<float>(y), u, v);
				tl_uf = std::min(tl_uf, u); tl_vf = std::min(tl_vf, v);
				br_uf = std::max(br_uf, u); br_vf = std::max(br_vf, v);
			}
		}

		dst_tl.x = static_cast<int>(tl_uf);
		dst_tl.y = static_cast<int>(tl_vf);
		dst_br.x = static_cast<int>(br_uf);
		dst_br.y = static_cast<int>(br_vf);
	}
    Mat xmap, ymap;
    xmap.create(dst_br.y - dst_tl.y + 1, dst_br.x - dst_tl.x + 1, CV_32F);
    ymap.create(dst_br.y - dst_tl.y + 1, dst_br.x - dst_tl.x + 1, CV_32F);
    
    float x, y;
    for (int v = dst_tl.y; v <= dst_br.y; ++v)
    {
        for (int u = dst_tl.x; u <= dst_br.x; ++u)
        {
            projector.mapBackward(static_cast<float>(u), static_cast<float>(v), x, y);
            //xmap.at<float>(v - dst_tl.y, u - dst_tl.x) = x+img_center.x;
            //ymap.at<float>(v - dst_tl.y, u - dst_tl.x) = y+img_center.y;
            xmap.at<float>(v - dst_tl.y, u - dst_tl.x) = x;
            ymap.at<float>(v - dst_tl.y, u - dst_tl.x) = y;
        }
    }
    
    cv::Rect dst_roi(dst_tl+map_center, dst_br+map_center+cv::Point(1,1));
    //cv::Mat dst(map, dst_roi);
    cv::Mat dst;
    dst.create(dst_roi.height, dst_roi.width, img.type());
    
    //std::cout << ymap << std::endl;
    cv::remap(img, dst, xmap, ymap, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
	
	cv::imshow("sub", dst);
    
	// copy roi to map
    cv::Mat map_roi(map, dst_roi);
	const uchar* sptr = dst.data;
	uchar* dptr = map_roi.data;
	// to handle the copying 1xn matrix => nx1 std vector.
	Size sz = dst.size();
	size_t len = sz.width*dst.elemSize();

	for( ; sz.height--; sptr += dst.step, dptr += map_roi.step )
		memcpy( dptr, sptr, len );

}
*/

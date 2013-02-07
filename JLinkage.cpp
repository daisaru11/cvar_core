//
//  JLinkage.h
//  cvar_core
//
//  Created by Daichi Sakai on 2013/01/13.
//  Copyright (c) 2013 Daichi Sakai. All rights reserved.
//
//  This is implementation vanishing lines clustering by J-Linkage algorithm.
//  See "J.-P. Tardif. Non-iterative approach for fast and accurate vanishing point detection."
//  And, we refered to the following link for the implementation.
//  http://srinathsridhar.com/research/eecs442

#include "JLinkage.h"

JLinkage::JLinkage()
{
}

JLinkage::~JLinkage()
{
}

void JLinkage::selectMinimalSets(const std::vector<Line>& edges)
{
	srand(time(NULL)); // Seed the rand function

	models.clear();
	models.resize(JLINKAGE_MODEL_SIZE);

	for(size_t i = 0; i < JLINKAGE_MODEL_SIZE; ++i)
	{
		Model& m = models[i];
		// For each modelsM choose, randomly, two lines
		size_t n1 = rand()%line_count;
		size_t n2 = rand()%line_count;
		while(n2 == n1)
			n2 = rand()%line_count;
		m.line1_idx = n1;
		m.line2_idx = n2;

		// Find vanishing point (intersection) for chosen model
		m.intersection_pt = findIntersection(edges[n1], edges[n2]);
	}
}

void JLinkage::makePSMatrix(const std::vector<Line>& edges)
{
	PSMatrix.clear();
	PSMatrix.resize(line_count);

	double distance;
	const double threshold = 2;

	for(size_t i = 0; i < line_count; i++)
	{
		for(size_t j = 0; j < JLINKAGE_MODEL_SIZE; j++)
		{
			// See proximity of ith line with jth model
			// Proximity in this case is perpendicular distance
			distance = findOrthDistance(edges[i], models[j].intersection_pt);
			if(distance <= threshold)
				PSMatrix[i][j] = 1;
			//else
				//PSMatrix[i][j] = 0;
		}
	}

}

void JLinkage::clusterPSMatrix(std::vector<std::vector<int> >& result)
{
  /* allocate memory */
	std::vector<std::vector<double> > distances(line_count, 
			std::vector<double>(line_count, 0));
	std::vector<std::vector<int> > clusters(line_count,
			std::vector<int>(line_count, 0));
	std::vector<int> indicators(line_count, 1);

	/* initialization */
	double minDis = 1;
	int indexA = 0, indexB = 0;
	for(int i = 0; i < line_count; i++)
	{
		clusters[i][i] = 1;
		for(int j = i + 1; j < line_count; j++)
		{
			const double jd = jaccardDist(PSMatrix[i], PSMatrix[j]);
			distances[i][j] = jd;
			distances[j][i] = jd;
			if(jd < minDis)
			{
				minDis = jd;
				indexA = i;
				indexB = j;
			}
		}
	}

	while(minDis != 1)
	{
		/* merge two clusters */
		for(int i = 0; i < line_count; i++)
		{
			if(clusters[indexA][i] || clusters[indexB][i])
				clusters[indexA][i] = clusters[indexB][i] = 1;
		}
		indicators[indexB] = 0;
		for(int i = 0; i < JLINKAGE_MODEL_SIZE; i++)
		{
			PSMatrix[indexA] = PSMatrix[indexB] = PSMatrix[indexA]&PSMatrix[indexB];
		}

		/* recalculate distance */
		for(int i = 0; i < line_count; i++)
		{
			distances[indexA][i] = jaccardDist(PSMatrix[indexA], PSMatrix[i]);
			distances[i][indexA] = distances[indexA][i];
		}

		/* find minimum distance */
		minDis = 1;
		for(int i = 0; i < line_count; i++)
		{
			if(indicators[i] == 0) continue;
			for(int j = i + 1; j < line_count; j++)
			{
				if(indicators[j] == 0) continue;
				if(distances[i][j] < minDis)
				{
					minDis = distances[i][j];
					indexA = i;
					indexB = j;
				}
			} 
		}
	}

 	/* calculate cluster size */
	std::vector<int> clusterSizes(line_count);
	for(int i = 0; i < line_count; i++)
	{
		int cs = 0;
		if(indicators[i])
		{
			for(int j = 0; j < line_count; j++)
			{
				if(clusters[i][j]) ++cs;
			}
		}
		clusterSizes[i] = cs;
	}

	const int cluster_num = 3;
	result.clear();
	result.resize(cluster_num, std::vector<int>(line_count)); /* choose the largest three clusters */


	int count = 0;
	while(count < cluster_num)
	{
		int max_index = 0;
		int max_size = clusterSizes[0];
		for(int i = 1; i < line_count; i++)
		{
			if(max_size < clusterSizes[i])
			{
				max_size = clusterSizes[i];
				max_index = i;
			}
		}
		result[count] = clusters[max_index];
		count++;
		clusterSizes[max_index] = 0;
	}

	/* print clusters */
	/*
	for(int i = 0; i < cluster_num; i++)
	{
		printf("Cluster %d:\n", i);
		for(int j = 0; j < line_count; j++)
		{
			if(result[i][j])
				printf("%d ", j);
		}
		printf("\n");
	}
	*/
}

cv::Point3d JLinkage::estimateVanPoint(const std::vector<Line>& edges, const std::vector<int>& cluster)
{
	int i;
	int num = 0;

	for(i = 0; i < line_count; i++)
	{
		if(cluster[i])
			num++;
	}

	cv::Mat A(num, 3, CV_64FC1);
	int count = 0;
	for(i = 0; i < line_count; i++)
	{
		if(cluster[i])
		{
			double l0 = edges[i].l[0];
			double l1 = edges[i].l[1];
			double l2 = edges[i].l[2];
			double nrm = sqrt(l0*l0 + l1*l1 + l2*l2);
			A.at<double>(count, 0) = l0 / nrm;
			A.at<double>(count, 1) = l1 / nrm;
			A.at<double>(count, 2) = l2 / nrm;
			count++;
		}
	}
	cv::SVD svd(A, cv::SVD::MODIFY_A|cv::SVD::FULL_UV);

	cv::Point3d vp(
			svd.vt.at<double>(2,0),
			svd.vt.at<double>(2,1),
			svd.vt.at<double>(2,2) );

	return vp;
}

void JLinkage::findVanPoints(const std::vector<Line>& edges, std::vector<std::vector<int> >& clusters, std::vector<cv::Point3d>& vps)
{
	clock_t start;

	// set line_count
	line_count = edges.size();
	printf("Number of edges is %d\n", line_count);
	
	if (line_count==0) {
		return;
	}

	// Select minimal sets
	start = clock();
	selectMinimalSets(edges);
	//printf("select minimal: %f\n", ((double)clock() - start) / CLOCKS_PER_SEC);

	// Construct Preference Set Matrix
	start = clock();
	makePSMatrix(edges);
	//printf("make ps mat: %f\n", ((double)clock() - start) / CLOCKS_PER_SEC);

	/*
	cv::Mat scaled(line_count, JLINKAGE_MODEL_SIZE, CV_8UC1);
	cv::Mat_<uchar> scaled_(scaled);
	for(size_t i = 0; i < line_count; i++)
	{
		for(size_t j = 0; j < JLINKAGE_MODEL_SIZE; j++)
		{
			if(PSMatrix[i][j] == true)
				scaled_(i, j) = 255;
			else
				scaled_(i, j) = 0;
		}
	}
	cv::imshow("PSMatrix", scaled);
	cv::waitKey();
	*/
	

	//Perform clustering on PSMatrix
	start = clock();
	clusterPSMatrix(clusters);
	//printf("cluster ps mat: %f\n", ((double)clock() - start) / CLOCKS_PER_SEC);

	start = clock();
	vps.clear();
	for(int i = 0; i < clusters.size(); i++)
		vps.push_back( estimateVanPoint(edges, clusters[i]) );
	//printf("estimate van: %f\n", ((double)clock() - start) / CLOCKS_PER_SEC);

}

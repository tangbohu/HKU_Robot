/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <rtabmap/core/RegistrationIcp.h>
#include <rtabmap/core/util3d_registration.h>
#include <rtabmap/core/util3d_surface.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UTimer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/conversions.h>

#ifdef RTABMAP_POINTMATCHER
#include "pointmatcher/PointMatcher.h"
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

DP pclToDP(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pclCloud)
{
	UDEBUG("");
	typedef DP::Label Label;
	typedef DP::Labels Labels;
	typedef DP::View View;

	if (pclCloud->empty())
		return DP();

	// fill labels
	// conversions of descriptor fields from pcl
	// see http://www.ros.org/wiki/pcl/Overview
	Labels featLabels;
	Labels descLabels;
	std::vector<bool> isFeature;
	featLabels.push_back(Label("x", 1));
	isFeature.push_back(true);
	featLabels.push_back(Label("y", 1));
	isFeature.push_back(true);
	featLabels.push_back(Label("z", 1));
	isFeature.push_back(true);
	featLabels.push_back(Label("pad", 1));

	// create cloud
	DP cloud(featLabels, descLabels, pclCloud->size());
	cloud.getFeatureViewByName("pad").setConstant(1);

	// fill cloud
	View viewX(cloud.getFeatureViewByName("x"));
	View viewY(cloud.getFeatureViewByName("y"));
	View viewZ(cloud.getFeatureViewByName("z"));
	for(unsigned int i=0; i<pclCloud->size(); ++i)
	{
		viewX(0, i) = pclCloud->at(i).x;
		viewY(0, i) = pclCloud->at(i).y;
		viewZ(0, i) = pclCloud->at(i).z;
	}

	return cloud;
}

DP pclToDP(const pcl::PointCloud<pcl::PointNormal>::Ptr & pclCloud)
{
	UDEBUG("");
	typedef DP::Label Label;
	typedef DP::Labels Labels;
	typedef DP::View View;

	if (pclCloud->empty())
		return DP();

	// fill labels
	// conversions of descriptor fields from pcl
	// see http://www.ros.org/wiki/pcl/Overview
	Labels featLabels;
	Labels descLabels;
	std::vector<bool> isFeature;
	featLabels.push_back(Label("x", 1));
	isFeature.push_back(true);
	featLabels.push_back(Label("y", 1));
	isFeature.push_back(true);
	featLabels.push_back(Label("z", 1));
	isFeature.push_back(true);

	descLabels.push_back(Label("normals", 3));
	isFeature.push_back(false);
	isFeature.push_back(false);
	isFeature.push_back(false);

	featLabels.push_back(Label("pad", 1));

	// create cloud
	DP cloud(featLabels, descLabels, pclCloud->size());
	cloud.getFeatureViewByName("pad").setConstant(1);

	// fill cloud
	View viewX(cloud.getFeatureViewByName("x"));
	View viewY(cloud.getFeatureViewByName("y"));
	View viewZ(cloud.getFeatureViewByName("z"));
	View viewNormalX(cloud.getDescriptorRowViewByName("normals",0));
	View viewNormalY(cloud.getDescriptorRowViewByName("normals",1));
	View viewNormalZ(cloud.getDescriptorRowViewByName("normals",2));
	for(unsigned int i=0; i<pclCloud->size(); ++i)
	{
		viewX(0, i) = pclCloud->at(i).x;
		viewY(0, i) = pclCloud->at(i).y;
		viewZ(0, i) = pclCloud->at(i).z;
		viewNormalX(0, i) = pclCloud->at(i).normal_x;
		viewNormalY(0, i) = pclCloud->at(i).normal_y;
		viewNormalZ(0, i) = pclCloud->at(i).normal_z;
	}

	return cloud;
}

void pclFromDP(const DP & cloud, pcl::PointCloud<pcl::PointXYZ> & pclCloud)
{
	UDEBUG("");
	typedef DP::ConstView ConstView;

	if (cloud.features.cols() == 0)
		return;

	pclCloud.resize(cloud.features.cols());
	pclCloud.is_dense = true;

		// fill cloud
	ConstView viewX(cloud.getFeatureViewByName("x"));
	ConstView viewY(cloud.getFeatureViewByName("y"));
	ConstView viewZ(cloud.getFeatureViewByName("z"));
	for(unsigned int i=0; i<pclCloud.size(); ++i)
	{
		pclCloud.at(i).x = viewX(0, i);
		pclCloud.at(i).y = viewY(0, i);
		pclCloud.at(i).z = viewZ(0, i);
	}
}

void pclFromDP(const DP & cloud, pcl::PointCloud<pcl::PointNormal> & pclCloud)
{
	UDEBUG("");
	typedef DP::ConstView ConstView;

	if (cloud.features.cols() == 0)
		return;

	pclCloud.resize(cloud.features.cols());
	pclCloud.is_dense = true;

		// fill cloud
	ConstView viewX(cloud.getFeatureViewByName("x"));
	ConstView viewY(cloud.getFeatureViewByName("y"));
	ConstView viewZ(cloud.getFeatureViewByName("z"));
	ConstView viewNormalX(cloud.getDescriptorRowViewByName("normals",0));
	ConstView viewNormalY(cloud.getDescriptorRowViewByName("normals",1));
	ConstView viewNormalZ(cloud.getDescriptorRowViewByName("normals",2));
	for(unsigned int i=0; i<pclCloud.size(); ++i)
	{
		pclCloud.at(i).x = viewX(0, i);
		pclCloud.at(i).y = viewY(0, i);
		pclCloud.at(i).z = viewZ(0, i);
		pclCloud.at(i).normal_x = viewNormalX(0, i);
		pclCloud.at(i).normal_y = viewNormalY(0, i);
		pclCloud.at(i).normal_z = viewNormalZ(0, i);
	}
}

template<typename T>
typename PointMatcher<T>::TransformationParameters eigenMatrixToDim(const typename PointMatcher<T>::TransformationParameters& matrix, int dimp1)
{
	typedef typename PointMatcher<T>::TransformationParameters M;
	assert(matrix.rows() == matrix.cols());
	assert((matrix.rows() == 3) || (matrix.rows() == 4));
	assert((dimp1 == 3) || (dimp1 == 4));

	if (matrix.rows() == dimp1)
		return matrix;

	M out(M::Identity(dimp1,dimp1));
	out.topLeftCorner(2,2) = matrix.topLeftCorner(2,2);
	out.topRightCorner(2,1) = matrix.topRightCorner(2,1);
	return out;
}

#endif

namespace rtabmap {

RegistrationIcp::RegistrationIcp(const ParametersMap & parameters, Registration * child) :
	Registration(parameters, child),
	_maxTranslation(Parameters::defaultIcpMaxTranslation()),
	_maxRotation(Parameters::defaultIcpMaxRotation()),
	_voxelSize(Parameters::defaultIcpVoxelSize()),
	_downsamplingStep(Parameters::defaultIcpDownsamplingStep()),
	_maxCorrespondenceDistance(Parameters::defaultIcpMaxCorrespondenceDistance()),
	_maxIterations(Parameters::defaultIcpIterations()),
	_epsilon(Parameters::defaultIcpEpsilon()),
	_correspondenceRatio(Parameters::defaultIcpCorrespondenceRatio()),
	_pointToPlane(Parameters::defaultIcpPointToPlane()),
	_pointToPlaneNormalNeighbors(Parameters::defaultIcpPointToPlaneNormalNeighbors()),
	_libpointmatcher(Parameters::defaultIcpPM()),
	_libpointmatcherConfig(Parameters::defaultIcpPMConfig()),
	_libpointmatcherOutlierRatio(Parameters::defaultIcpPMOutlierRatio()),
	_libpointmatcherICP(0)
{
	this->parseParameters(parameters);
}

RegistrationIcp::~RegistrationIcp()
{
#ifdef RTABMAP_POINTMATCHER
	if(_libpointmatcherICP)
	{
		delete (PM::ICP*)_libpointmatcherICP;
	}
#endif
}

void RegistrationIcp::parseParameters(const ParametersMap & parameters)
{
	Registration::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kIcpMaxTranslation(), _maxTranslation);
	Parameters::parse(parameters, Parameters::kIcpMaxRotation(), _maxRotation);
	Parameters::parse(parameters, Parameters::kIcpVoxelSize(), _voxelSize);
	Parameters::parse(parameters, Parameters::kIcpDownsamplingStep(), _downsamplingStep);
	Parameters::parse(parameters, Parameters::kIcpMaxCorrespondenceDistance(), _maxCorrespondenceDistance);
	Parameters::parse(parameters, Parameters::kIcpIterations(), _maxIterations);
	Parameters::parse(parameters, Parameters::kIcpEpsilon(), _epsilon);
	Parameters::parse(parameters, Parameters::kIcpCorrespondenceRatio(), _correspondenceRatio);
	Parameters::parse(parameters, Parameters::kIcpPointToPlane(), _pointToPlane);
	Parameters::parse(parameters, Parameters::kIcpPointToPlaneNormalNeighbors(), _pointToPlaneNormalNeighbors);

	Parameters::parse(parameters, Parameters::kIcpPM(), _libpointmatcher);
	Parameters::parse(parameters, Parameters::kIcpPMConfig(), _libpointmatcherConfig);
	Parameters::parse(parameters, Parameters::kIcpPMOutlierRatio(), _libpointmatcherOutlierRatio);

#ifndef RTABMAP_POINTMATCHER
	if(_libpointmatcher)
	{
		UWARN("Parameter %s is set to true but RTAB-MAp has not been built with libpointmatcher support. Setting to false.", Parameters::kIcpPM().c_str());
		_libpointmatcher = false;
	}
#else
	if(_libpointmatcher)
	{
		UINFO("libpointmatcher enabled! config=\"%s\"", _libpointmatcherConfig.c_str());
		if(_libpointmatcherICP!=0)
		{
			delete (PM::ICP*)_libpointmatcherICP;
			_libpointmatcherICP = 0;
		}

		_libpointmatcherICP = new PM::ICP();

		PM::ICP * icp = (PM::ICP*)_libpointmatcherICP;

		bool useDefaults = true;
		if(!_libpointmatcherConfig.empty())
		{
			// load YAML config
			std::ifstream ifs(_libpointmatcherConfig.c_str());
			if (ifs.good())
			{
				icp->loadFromYaml(ifs);
				useDefaults = false;
			}
			else
			{
				UERROR("Cannot open libpointmatcher config file \"%s\", using default values instead.", _libpointmatcherConfig.c_str());
			}
		}
		if(useDefaults)
		{
			// Create the default ICP algorithm
			// See the implementation of setDefault() to create a custom ICP algorithm
			icp->setDefault();

			icp->readingDataPointsFilters.clear();
			icp->readingDataPointsFilters.push_back(PM::get().DataPointsFilterRegistrar.create("IdentityDataPointsFilter"));

			icp->referenceDataPointsFilters.clear();
			icp->referenceDataPointsFilters.push_back(PM::get().DataPointsFilterRegistrar.create("IdentityDataPointsFilter"));

			PM::Parameters params;
			params["maxDist"] = uNumber2Str(_maxCorrespondenceDistance);
			icp->matcher.reset(PM::get().MatcherRegistrar.create("KDTreeMatcher", params));
			params.clear();

			params["ratio"] = uNumber2Str(_libpointmatcherOutlierRatio);
			icp->outlierFilters.clear();
			icp->outlierFilters.push_back(PM::get().OutlierFilterRegistrar.create("TrimmedDistOutlierFilter", params));
			params.clear();

			icp->errorMinimizer.reset(PM::get().ErrorMinimizerRegistrar.create(_pointToPlane?"PointToPlaneErrorMinimizer":"PointToPointErrorMinimizer"));

			icp->transformationCheckers.clear();
			params["maxIterationCount"] = uNumber2Str(_maxIterations);
			icp->transformationCheckers.push_back(PM::get().TransformationCheckerRegistrar.create("CounterTransformationChecker", params));
			params.clear();

			params["minDiffRotErr"] =   uNumber2Str(_epsilon*_epsilon*100.0f);
			params["minDiffTransErr"] = uNumber2Str(_epsilon*_epsilon);
			params["smoothLength"] =    uNumber2Str(4);
			icp->transformationCheckers.push_back(PM::get().TransformationCheckerRegistrar.create("DifferentialTransformationChecker", params));
			params.clear();
		}
	}
#endif

	UASSERT_MSG(_voxelSize >= 0, uFormat("value=%d", _voxelSize).c_str());
	UASSERT_MSG(_downsamplingStep >= 0, uFormat("value=%d", _downsamplingStep).c_str());
	UASSERT_MSG(_maxCorrespondenceDistance > 0.0f, uFormat("value=%f", _maxCorrespondenceDistance).c_str());
	UASSERT_MSG(_maxIterations > 0, uFormat("value=%d", _maxIterations).c_str());
	UASSERT(_epsilon >= 0.0f);
	UASSERT_MSG(_correspondenceRatio >=0.0f && _correspondenceRatio <=1.0f, uFormat("value=%f", _correspondenceRatio).c_str());
	UASSERT_MSG(_pointToPlaneNormalNeighbors > 0, uFormat("value=%d", _pointToPlaneNormalNeighbors).c_str());
}

Transform RegistrationIcp::computeTransformationImpl(
			Signature & fromSignature,
			Signature & toSignature,
			Transform guess,
			RegistrationInfo & info) const
{
	UDEBUG("Guess transform = %s", guess.prettyPrint().c_str());
	UDEBUG("Voxel size=%f", _voxelSize);
	UDEBUG("PointToPlane=%d", _pointToPlane?1:0);
	UDEBUG("Normal neighborhood=%d", _pointToPlaneNormalNeighbors);
	UDEBUG("Max correspondence distance=%f", _maxCorrespondenceDistance);
	UDEBUG("Max Iterations=%d", _maxIterations);
	UDEBUG("Correspondence Ratio=%f", _correspondenceRatio);
	UDEBUG("Max translation=%f", _maxTranslation);
	UDEBUG("Max rotation=%f", _maxRotation);
	UDEBUG("Downsampling step=%d", _downsamplingStep);
	UDEBUG("libpointmatcher=%d (outlier ratio=%f)", _libpointmatcher?1:0, _libpointmatcherOutlierRatio);

	UTimer timer;
	std::string msg;
	Transform transform;

	SensorData & dataFrom = fromSignature.sensorData();
	SensorData & dataTo = toSignature.sensorData();

	UDEBUG("size from=%d (channels=%d, max pts=%d) to=%d (channels=%d, max pts=%d)",
			dataFrom.laserScanRaw().cols,
			dataFrom.laserScanRaw().channels(),
			dataFrom.laserScanInfo().maxPoints(),
			dataTo.laserScanRaw().cols,
			dataTo.laserScanRaw().channels(),
			dataTo.laserScanInfo().maxPoints());

	if(!guess.isNull() && !dataFrom.laserScanRaw().empty() && !dataTo.laserScanRaw().empty())
	{
		// ICP with guess transform
		int maxLaserScansTo = dataTo.laserScanInfo().maxPoints();
		int maxLaserScansFrom = dataFrom.laserScanInfo().maxPoints();
		cv::Mat fromScan = dataFrom.laserScanRaw();
		cv::Mat toScan = dataTo.laserScanRaw();
		Transform fromLocalTransform = dataFrom.laserScanInfo().localTransform();
		Transform toLocalTransform = dataTo.laserScanInfo().localTransform();
		if(_downsamplingStep>1)
		{
			fromScan = util3d::downsample(fromScan, _downsamplingStep);
			toScan = util3d::downsample(toScan, _downsamplingStep);
			maxLaserScansTo/=_downsamplingStep;
			maxLaserScansFrom/=_downsamplingStep;
			UDEBUG("Downsampling time (step=%d) = %f s", _downsamplingStep, timer.ticks());
		}

		if(fromScan.cols && toScan.cols)
		{
			Transform icpT;
			bool hasConverged = false;
			float correspondencesRatio = 0.0f;
			int correspondences = 0;
			double variance = 1.0;

			if( _pointToPlane &&
				_voxelSize == 0.0f &&
				fromScan.channels() == 6 &&
				toScan.channels() == 6)
			{
				//special case if we have already normals computed and there is no filtering
				pcl::PointCloud<pcl::PointNormal>::Ptr fromCloudNormals = util3d::laserScanToPointCloudNormal(fromScan, fromLocalTransform);
				pcl::PointCloud<pcl::PointNormal>::Ptr toCloudNormals = util3d::laserScanToPointCloudNormal(toScan, guess * toLocalTransform);

				UDEBUG("Conversion time = %f s", timer.ticks());
				pcl::PointCloud<pcl::PointNormal>::Ptr fromCloudNormalsRegistered(new pcl::PointCloud<pcl::PointNormal>());
#ifdef RTABMAP_POINTMATCHER
				if(_libpointmatcher)
				{
					// Load point clouds
					DP data = pclToDP(fromCloudNormals);
					DP ref = pclToDP(toCloudNormals);

					// Compute the transformation to express data in ref
					PM::TransformationParameters T;
					try
					{
						UASSERT(_libpointmatcherICP != 0);
						PM::ICP & icp = *((PM::ICP*)_libpointmatcherICP);
						UDEBUG("libpointmatcher icp... (if there is a seg fault here, make sure all third party libraries are built with same Eigen version.)");
						T = icp(data, ref);
						UDEBUG("libpointmatcher icp...done!");
						icpT = Transform::fromEigen3d(Eigen::Affine3d(Eigen::Matrix4d(eigenMatrixToDim<double>(T.template cast<double>(), 4))));

						float matchRatio = icp.errorMinimizer->getWeightedPointUsedRatio();
						UDEBUG("match ratio: %f", matchRatio);

						if(!icpT.isNull())
						{
							fromCloudNormalsRegistered = util3d::transformPointCloud(fromCloudNormals, icpT);
							hasConverged = true;
						}
					}
					catch(const std::exception & e)
					{
						UWARN("libpointmatcher has failed: %s", e.what());
					}
				}
				else
#endif
				{
					icpT = util3d::icpPointToPlane(
							fromCloudNormals,
							toCloudNormals,
						   _maxCorrespondenceDistance,
						   _maxIterations,
						   hasConverged,
						   *fromCloudNormalsRegistered,
						   _epsilon,
						   this->force3DoF());
				}

				if(!icpT.isNull() && hasConverged)
				{
					util3d::computeVarianceAndCorrespondences(
							fromCloudNormalsRegistered,
							toCloudNormals,
							_maxCorrespondenceDistance,
							variance,
							correspondences);
				}
			}
			else
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr fromCloud = util3d::laserScanToPointCloud(fromScan, fromLocalTransform);
				pcl::PointCloud<pcl::PointXYZ>::Ptr toCloud = util3d::laserScanToPointCloud(toScan, guess * toLocalTransform);
				UDEBUG("Conversion time = %f s", timer.ticks());

				pcl::PointCloud<pcl::PointXYZ>::Ptr fromCloudFiltered = fromCloud;
				pcl::PointCloud<pcl::PointXYZ>::Ptr toCloudFiltered = toCloud;
				if(_voxelSize > 0.0f)
				{
					int pointsBeforeFiltering = fromCloudFiltered->size();
					fromCloudFiltered = util3d::voxelize(fromCloudFiltered, _voxelSize);
					maxLaserScansFrom = maxLaserScansFrom * fromCloudFiltered->size() / pointsBeforeFiltering;

					pointsBeforeFiltering = toCloudFiltered->size();
					toCloudFiltered = util3d::voxelize(toCloudFiltered, _voxelSize);
					maxLaserScansTo = maxLaserScansTo * toCloudFiltered->size() / pointsBeforeFiltering;

					UDEBUG("Voxel filtering time (voxel=%f m, ratioFrom=%f ratioTo=%f) = %f s",
							_voxelSize,
							float(fromCloudFiltered->size()) / float(pointsBeforeFiltering),
							float(toCloudFiltered->size()) / float(pointsBeforeFiltering),
							timer.ticks());
				}

				pcl::PointCloud<pcl::PointXYZ>::Ptr fromCloudRegistered(new pcl::PointCloud<pcl::PointXYZ>());
				if(_pointToPlane) // ICP Point To Plane, only in 3D
				{
					pcl::PointCloud<pcl::Normal>::Ptr normals;

					normals = util3d::computeNormals(fromCloudFiltered, _pointToPlaneNormalNeighbors);
					pcl::PointCloud<pcl::PointNormal>::Ptr fromCloudNormals(new pcl::PointCloud<pcl::PointNormal>);
					pcl::concatenateFields(*fromCloudFiltered, *normals, *fromCloudNormals);

					normals = util3d::computeNormals(toCloudFiltered, _pointToPlaneNormalNeighbors);
					pcl::PointCloud<pcl::PointNormal>::Ptr toCloudNormals(new pcl::PointCloud<pcl::PointNormal>);
					pcl::concatenateFields(*toCloudFiltered, *normals, *toCloudNormals);

					std::vector<int> indices;
					toCloudNormals = util3d::removeNaNNormalsFromPointCloud(toCloudNormals);
					fromCloudNormals = util3d::removeNaNNormalsFromPointCloud(fromCloudNormals);

					// update output scans
					fromSignature.sensorData().setLaserScanRaw(util3d::laserScanFromPointCloud(*fromCloudNormals, fromLocalTransform.inverse()), LaserScanInfo(maxLaserScansFrom, fromSignature.sensorData().laserScanInfo().maxRange(), fromLocalTransform));
					toSignature.sensorData().setLaserScanRaw(util3d::laserScanFromPointCloud(*toCloudNormals, (guess*toLocalTransform).inverse()), LaserScanInfo(maxLaserScansTo, toSignature.sensorData().laserScanInfo().maxRange(), toLocalTransform));

					UDEBUG("Compute normals time = %f s", timer.ticks());

					if(toCloudNormals->size() && fromCloudNormals->size())
					{
						pcl::PointCloud<pcl::PointNormal>::Ptr fromCloudNormalsRegistered(new pcl::PointCloud<pcl::PointNormal>());

#ifdef RTABMAP_POINTMATCHER
						if(_libpointmatcher)
						{
							// Load point clouds
							DP data = pclToDP(fromCloudNormals);
							DP ref = pclToDP(toCloudNormals);

							// Compute the transformation to express data in ref
							PM::TransformationParameters T;
							try
							{
								UASSERT(_libpointmatcherICP != 0);
								PM::ICP & icp = *((PM::ICP*)_libpointmatcherICP);
								UDEBUG("libpointmatcher icp... (if there is a seg fault here, make sure all third party libraries are built with same Eigen version.)");
								T = icp(data, ref);
								UDEBUG("libpointmatcher icp...done!");
								icpT = Transform::fromEigen3d(Eigen::Affine3d(Eigen::Matrix4d(eigenMatrixToDim<double>(T.template cast<double>(), 4))));

								float matchRatio = icp.errorMinimizer->getWeightedPointUsedRatio();
								UDEBUG("match ratio: %f", matchRatio);

								if(!icpT.isNull())
								{
									fromCloudNormalsRegistered = util3d::transformPointCloud(fromCloudNormals, icpT);
									hasConverged = true;
								}
							}
							catch(const std::exception & e)
							{
								UWARN("libpointmatcher has failed: %s", e.what());
							}
						}
						else
#endif
						{
							icpT = util3d::icpPointToPlane(
									fromCloudNormals,
									toCloudNormals,
								   _maxCorrespondenceDistance,
								   _maxIterations,
								   hasConverged,
								   *fromCloudNormalsRegistered,
								   _epsilon,
								   this->force3DoF());
						}


						if(!icpT.isNull() && hasConverged)
						{
							util3d::computeVarianceAndCorrespondences(
									fromCloudNormalsRegistered,
									toCloudNormals,
									_maxCorrespondenceDistance,
									variance,
									correspondences);
						}
					}
				}
				else // ICP Point to Point
				{
					if(_voxelSize > 0.0f)
					{
						// update output scans
						fromSignature.sensorData().setLaserScanRaw(util3d::laserScanFromPointCloud(*fromCloudFiltered, fromLocalTransform.inverse()), LaserScanInfo(maxLaserScansFrom, fromSignature.sensorData().laserScanInfo().maxRange(), fromLocalTransform));
						toSignature.sensorData().setLaserScanRaw(util3d::laserScanFromPointCloud(*toCloudFiltered, (guess*toLocalTransform).inverse()), LaserScanInfo(maxLaserScansTo, toSignature.sensorData().laserScanInfo().maxRange(), toLocalTransform));
					}

#ifdef RTABMAP_POINTMATCHER
					if(_libpointmatcher)
					{
						// Load point clouds
						DP data = pclToDP(fromCloudFiltered);
						DP ref = pclToDP(toCloudFiltered);

						// Compute the transformation to express data in ref
						PM::TransformationParameters T;
						try
						{
							UASSERT(_libpointmatcherICP != 0);
							PM::ICP & icp = *((PM::ICP*)_libpointmatcherICP);
							UDEBUG("libpointmatcher icp... (if there is a seg fault here, make sure all third party libraries are built with same Eigen version.)");
							T = icp(data, ref);
							UDEBUG("libpointmatcher icp...done!");
							icpT = Transform::fromEigen3d(Eigen::Affine3d(Eigen::Matrix4d(eigenMatrixToDim<double>(T.template cast<double>(), 4))));

							float matchRatio = icp.errorMinimizer->getWeightedPointUsedRatio();
							UDEBUG("match ratio: %f", matchRatio);

							if(!icpT.isNull())
							{
								fromCloudRegistered = util3d::transformPointCloud(fromCloudFiltered, icpT);
								hasConverged = true;
							}
						}
						catch(const std::exception & e)
						{
							UWARN("libpointmatcher has failed: %s", e.what());
						}
					}
					else
#endif
					{
						icpT = util3d::icp(
								fromCloudFiltered,
								toCloudFiltered,
							   _maxCorrespondenceDistance,
							   _maxIterations,
							   hasConverged,
							   *fromCloudRegistered,
							   _epsilon,
							   this->force3DoF()); // icp2D
					}

					if(!icpT.isNull() && hasConverged)
					{
						util3d::computeVarianceAndCorrespondences(
								fromCloudRegistered,
								toCloudFiltered,
								_maxCorrespondenceDistance,
								variance,
								correspondences);
					}
				}
			}
			UDEBUG("ICP (iterations=%d) time = %f s", _maxIterations, timer.ticks());

			if(!icpT.isNull() && hasConverged)
			{
				float ix,iy,iz, iroll,ipitch,iyaw;
				Transform icpInTargetReferential = guess.inverse() * icpT.inverse() * guess; // actual local ICP refinement
				icpInTargetReferential.getTranslationAndEulerAngles(ix,iy,iz,iroll,ipitch,iyaw);
				info.icpTranslation = uMax3(fabs(ix), fabs(iy), fabs(iz));
				info.icpRotation = uMax3(fabs(iroll), fabs(ipitch), fabs(iyaw));
				if((_maxTranslation>0.0f &&
						info.icpTranslation > _maxTranslation)
				   ||
				   (_maxRotation>0.0f &&
						info.icpRotation > _maxRotation))
				{
					msg = uFormat("Cannot compute transform (ICP correction too large -> %f m %f rad, limits=%f m, %f rad)",
							info.icpTranslation,
							info.icpRotation,
							_maxTranslation,
							_maxRotation);
					UINFO(msg.c_str());
				}
				else
				{
					// verify if there are enough correspondences (using "To" by default if set, in case if "From" is merged from multiple scans)
					int maxLaserScans = maxLaserScansTo?maxLaserScansTo:maxLaserScansFrom;
					UDEBUG("Max scans=%d (from=%d, to=%d)", maxLaserScans, maxLaserScansFrom, maxLaserScansTo);
					if(maxLaserScans)
					{
						correspondencesRatio = float(correspondences)/float(maxLaserScans);
					}
					else
					{
						static bool warningShown = false;
						if(!warningShown)
						{
							UWARN("Maximum laser scans points not set for signature %d, correspondences ratio set relative instead of absolute! This message will only appear once.",
									dataTo.id());
							warningShown = true;
						}
						correspondencesRatio = float(correspondences)/float(toScan.cols>fromScan.cols?toScan.cols:fromScan.cols);
					}

					UDEBUG("%d->%d hasConverged=%s, variance=%f, correspondences=%d/%d (%f%%), from guess: trans=%f rot=%f",
							dataTo.id(), dataFrom.id(),
							hasConverged?"true":"false",
							variance,
							correspondences,
							maxLaserScans>0?maxLaserScans:(int)(toScan.cols>fromScan.cols?toScan.cols:fromScan.cols),
							correspondencesRatio*100.0f,
							info.icpTranslation,
							info.icpRotation);

					info.covariance = cv::Mat::eye(6,6,CV_64FC1)*(variance>0.0001?variance:0.0001); // epsilon if exact transform
					info.icpInliersRatio = correspondencesRatio;

					if(correspondencesRatio < _correspondenceRatio)
					{
						msg = uFormat("Cannot compute transform (cor=%d corrRatio=%f/%f)",
								correspondences, correspondencesRatio, _correspondenceRatio);
						UINFO(msg.c_str());
					}
					else
					{
						transform = icpT.inverse()*guess;
					}
				}
			}
			else
			{
				msg = uFormat("Cannot compute transform (converged=%s var=%f)",
						hasConverged?"true":"false", variance);
				UINFO(msg.c_str());
			}
		}
		else
		{
			msg = "Laser scans empty ?!?";
			UWARN(msg.c_str());
		}
	}
	else if(dataTo.isValid())
	{
		if(guess.isNull())
		{
			msg = "RegistrationIcp cannot do registration with a null guess.";
		}
		else
		{
			msg = uFormat("Laser scans empty?!? (new[%d]=%d old[%d]=%d)",
					dataTo.id(), dataTo.laserScanRaw().total(),
					dataFrom.id(), dataFrom.laserScanRaw().total());
		}
		UERROR(msg.c_str());
	}


	info.rejectedMsg = msg;

	UDEBUG("New transform = %s", transform.prettyPrint().c_str());
	return transform;
}

}

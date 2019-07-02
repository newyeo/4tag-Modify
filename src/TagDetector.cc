#include <algorithm>
#include <cmath>
#include <climits>
#include <map>
#include <vector>
#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include "AprilTags/Edge.h"
#include "AprilTags/FloatImage.h"
#include "AprilTags/Gaussian.h"
#include "AprilTags/GrayModel.h"
#include "AprilTags/GLine2D.h"
#include "AprilTags/GLineSegment2D.h"
#include "AprilTags/Gridder.h"
#include "AprilTags/Homography33.h"
#include "AprilTags/MathUtil.h"
#include "AprilTags/Quad.h"
#include "AprilTags/Segment.h"
#include "AprilTags/TagFamily.h"
#include "AprilTags/UnionFindSimple.h"
#include "AprilTags/XYWeight.h"

#include "AprilTags/TagDetector.h"

#include <sys/time.h>

//#define DEBUG_APRIL

#ifdef DEBUG_APRIL
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

//#define FIND_RECT


#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/opencv.hpp"

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

//#include <cmath>
//#include <cstdlib>
//#include <random>
//#include <stdlib.h>


using namespace std;
using namespace cv;

#ifdef FIND_RECT
struct  Center
{
    Point2d location;
    double radius;
    double confidence;
};

SimpleBlobDetector::Params params;

void findBlobs(const cv::Mat &image, const cv::Mat &binaryImage, vector<Center> &centers)
{
    (void)image;
    centers.clear();

    vector < vector<Point> > contours;
    Mat tmpBinaryImage = binaryImage.clone();
    findContours(tmpBinaryImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    for (size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
    {
        Center center;
        center.confidence = 1;
        Moments moms = moments(Mat(contours[contourIdx]));
        if (params.filterByArea)
        {
            double area = moms.m00;
            if (area < params.minArea || area >= params.maxArea)
                continue;
        }

        if (params.filterByCircularity)
        {
            double area = moms.m00;
            double perimeter = arcLength(Mat(contours[contourIdx]), true);
            double ratio = 4 * CV_PI * area / (perimeter * perimeter);
            if (ratio < params.minCircularity || ratio >= params.maxCircularity)
                continue;
        }

        if (params.filterByInertia)
        {
            double denominator = sqrt(pow(2 * moms.mu11, 2) + pow(moms.mu20 - moms.mu02, 2));
            const double eps = 1e-2;
            double ratio;
            if (denominator > eps)
            {
                double cosmin = (moms.mu20 - moms.mu02) / denominator;
                double sinmin = 2 * moms.mu11 / denominator;
                double cosmax = -cosmin;
                double sinmax = -sinmin;

                double imin = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmin - moms.mu11 * sinmin;
                double imax = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmax - moms.mu11 * sinmax;
                ratio = imin / imax;
            }
            else
            {
                ratio = 1;
            }

            if (ratio < params.minInertiaRatio || ratio >= params.maxInertiaRatio)
                continue;

            center.confidence = ratio * ratio;
        }

        if (params.filterByConvexity)
        {
            vector < Point > hull;
            convexHull(Mat(contours[contourIdx]), hull);
            double area = contourArea(Mat(contours[contourIdx]));
            double hullArea = contourArea(Mat(hull));
            double ratio = area / hullArea;
            if (ratio < params.minConvexity || ratio >= params.maxConvexity)
                continue;
        }

        center.location = Point2d(moms.m10 / moms.m00, moms.m01 / moms.m00);

        if (params.filterByColor)
        {
            if (binaryImage.at<uchar> (cvRound(center.location.y), cvRound(center.location.x)) != params.blobColor)
                continue;
        }

        //compute blob radius
        {
            vector<double> dists;
            for (size_t pointIdx = 0; pointIdx < contours[contourIdx].size(); pointIdx++)
            {
                Point2d pt = contours[contourIdx][pointIdx];
                dists.push_back(norm(center.location - pt));
            }
            std::sort(dists.begin(), dists.end());
            center.radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.;
        }

        centers.push_back(center);

#ifdef DEBUG_BLOB_DETECTOR
        //    circle( keypointsImage, center.location, 1, Scalar(0,0,255), 1 );
#endif
    }
#ifdef DEBUG_BLOB_DETECTOR
    //  imshow("bk", keypointsImage );
    //  waitKey();
#endif
}
/*
//distance -- max distance to the random line for voting
//ngon     -- n-gon to be detected
//itmax    -- max iteration times
void ransacLines(std::vector<cv::Point2f>& input,std::vector<cv::Vec4d>& lines,
                    double distance ,  unsigned int ngon,unsigned int itmax ){

    if(!input.empty())
    for(int i = 0; i < ngon; ++i)
    {
        unsigned int Mmax = 0;
        cv::Point imax;
        cv::Point jmax;
        cv::Vec4d line;
        size_t t1 , t2;
        
        std::random_device rd;     // only used once to initialise (seed) engine
        std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
        std::uniform_int_distribution<int> uni(0,input.size()-1); // guaranteed unbiased // 概率相同

        unsigned int it = itmax;
        while(--it){
            t1 = uni(rng);
            t2 = uni(rng);
            t2 = (t1 == t2 ? uni(rng): t2);
            unsigned int M = 0;
            cv::Point i = input[t1];
            cv::Point j = input[t2];
            for(auto a : input){
                double dis = fabs((j.x - i.x)*(a.y - i.y) - (j.y - i.y)*(a.x - i.x)) /
                        sqrt((j.x - i.x)*(j.x - i.x) + (j.y - i.y)*(j.y - i.y));

                if( dis < distance)
                    ++M;
            }
            if(M > Mmax ){
                Mmax = M;
                imax = i;
                jmax = j;
            }
        }
        line[0] = imax.x;
        line[1] = imax.y;
        line[2] = jmax.x;
        line[3] = jmax.y;
        lines.push_back(line);
        auto iter = input.begin();
        while(iter != input.end()){
            double dis = fabs((jmax.x - imax.x)*((*iter).y - imax.y) -
                                    (jmax.y - imax.y)*((*iter).x - imax.x))
                         / sqrt((jmax.x - imax.x)*(jmax.x - imax.x)
                                 + (jmax.y - imax.y)*(jmax.y - imax.y));
            if(dis < distance)
                iter = input.erase(iter);  //erase the dis within , then point to
                                           //   the next element
            else ++iter;
        }
    }
    else std::cout << "no input to ransacLines" << std::endl;
}
*/
int num_save = 0;
#endif

//#define DEBUG_APRIL

namespace AprilTags {

    const float PI =3.14159265354;
    const int   PERIMETER_THRESHOLD=250;
    const string preFixPath="/home/yao/nansha/test_label/";

    std::vector<TagDetection> TagDetector::extractTags(const cv::Mat &image_src, int index_up_down) {

        cv::Mat image_color, image, im_gray;
        image_src.copyTo(image_color);

        cv::Mat dst_laplacian, tmp_mean, tmp_std;
        cv::cvtColor(image_color, image, cv::COLOR_RGB2GRAY);
//        cv::Laplacian(im_gray,dst_laplacian, CV_16U, 3);
//        cv::meanStdDev(dst_laplacian, tmp_mean, tmp_std);
//        double mean=tmp_mean.at<double>(0,0);
//        double std =tmp_std.at<double>(0,0);
//
//        //********preprocessing************
//        if(std*std<40)
//        {
//            std::cout << "mean:"<<mean<<", std:"<<std*std<<std::endl;
//            cv::Mat kernel(3,3,CV_32F,cv::Scalar(-1));
//            kernel.at<float>(1,1)=8.9;
//            cv::filter2D(im_gray,image,image_color.depth(),kernel);
//        } else{
//            im_gray.copyTo(image);
//        }

        //=============end=================

        std::vector<TagDetection> goodDetections;

        double timeuse;

        struct timeval begin_extractTags;
        gettimeofday(&begin_extractTags, NULL);

        // convert to internal AprilTags image (todo: slow, change internally to OpenCV)
        int width = image.cols;
        int height = image.rows;
        AprilTags::FloatImage fimOrig(width, height);
        int i = 0;
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                fimOrig.set(x, y, image.data[i] / 255.);
                i++;
            }
        }
        std::pair<int, int> opticalCenter(width / 2, height / 2);



        //================================================================
        // Step one: preprocess image (convert to grayscale) and low pass if necessary

        FloatImage fim = fimOrig;

        //! Gaussian smoothing kernel applied to image (0 == no filter).
        /*! Used when sampling bits. Filtering is a good idea in cases
         * where A) a cheap camera is introducing artifical sharpening, B)
         * the bayer pattern is creating artifcats, C) the sensor is very
         * noisy and/or has hot/cold pixels. However, filtering makes it
         * harder to decode very small tags. Reasonable values are 0, or
         * [0.8, 1.5].
         */
        float sigma = 0;

        //! Gaussian smoothing kernel applied to image (0 == no filter).
        /*! Used when detecting the outline of the box. It is almost always
         * useful to have some filtering, since the loss of small details
         * won't hurt. Recommended value = 0.8. The case where sigma ==
         * segsigma has been optimized to avoid a redundant filter
         * operation.
         */
        float segSigma = 0.8f;

        if (sigma > 0) {
            int filtsz = ((int) max(3.0f, 3 * sigma)) | 1;
            std::vector<float> filt = Gaussian::makeGaussianFilter(sigma, filtsz);
            fim.filterFactoredCentered(filt, filt);
        }

        struct timeval Step_two_extractTags;
        gettimeofday(&Step_two_extractTags, NULL);

        timeuse = 1000000 * (Step_two_extractTags.tv_sec - begin_extractTags.tv_sec) + Step_two_extractTags.tv_usec -
                  begin_extractTags.tv_usec;
        timeuse /= 1000.0;
        //if(timeuse >= 25 && index_up_down == 1)
        //return goodDetections;

        //================================================================
        // Step two: Compute the local gradient. We store the direction and magnitude.
        // This step is quite sensitve to noise, since a few bad theta estimates will
        // break up segments, causing us to miss Quads. It is useful to do a Gaussian
        // low pass on this step even if we don't want it for encoding.

        FloatImage fimSeg;
        if (segSigma > 0) {
            if (segSigma == sigma) {
                fimSeg = fim;
            } else {
                // blur anew
                int filtsz = ((int) max(3.0f, 3 * segSigma)) | 1;
                std::vector<float> filt = Gaussian::makeGaussianFilter(segSigma, filtsz);
                fimSeg = fimOrig;
                fimSeg.filterFactoredCentered(filt, filt);
            }
        } else {
            fimSeg = fimOrig;
        }

        FloatImage fimTheta(fimSeg.getWidth(), fimSeg.getHeight());
        FloatImage fimMag(fimSeg.getWidth(), fimSeg.getHeight());


#pragma omp parallel for
        //cout<<fimSeg.getHeight()<<", "<<fimSeg.getWidth()<<endl;
        for (int y = 1; y < fimSeg.getHeight() - 1; y++) {
            for (int x = 1; x < fimSeg.getWidth() - 1; x++) {
                float Ix = fimSeg.get(x + 1, y) - fimSeg.get(x - 1, y);
                float Iy = fimSeg.get(x, y + 1) - fimSeg.get(x, y - 1);

                float mag = Ix * Ix + Iy * Iy;
#if 0 // kaess: fast version, but maybe less accurate?
                float theta = MathUtil::fast_atan2(Iy, Ix);
#else
                float theta = atan2(Iy, Ix);
#endif

                fimTheta.set(x, y, theta);
                fimMag.set(x, y, mag);
            }
        }

#ifdef DEBUG_APRIL
        int height_ = fimSeg.getHeight();
        int width_ = fimSeg.getWidth();
        cv::Mat image_text(height_, width_, CV_8UC3);
        {
            for (int y = 0; y < height_; y++) {
                for (int x = 0; x < width_; x++) {
                    cv::Vec3b v;
                    //        float vf = fimMag.get(x,y);
                    float vf = fimOrig.get(x, y);
                    int val = (int) (vf * 255.);
                    if ((val & 0xffff00) != 0) { printf("problem... %i\n", val); }
                    for (int k = 0; k < 3; k++) {
                        v(k) = val;
                    }
                    image_text.at<cv::Vec3b>(y, x) = v;
                }
            }
        }
//        cv::imshow("debug_april_text", image_text);
//  waitKey(10);
#endif

        struct timeval Step_three_extractTags;
        gettimeofday(&Step_three_extractTags, NULL);

        timeuse =
                1000000 * (Step_three_extractTags.tv_sec - begin_extractTags.tv_sec) + Step_three_extractTags.tv_usec -
                begin_extractTags.tv_usec;
        timeuse /= 1000.0;
        //if(timeuse >= 25 && index_up_down == 1)
        //return goodDetections;
        //================================================================
        // Step three. Extract edges by grouping pixels with similar
        // thetas together. This is a greedy algorithm: we start with
        // the most similar pixels.  We use 4-connectivity.
/*
  struct timeval tpstart1;
  float timeuse1;
  gettimeofday(&tpstart1, NULL);
*/
        UnionFindSimple uf(fimSeg.getWidth() * fimSeg.getHeight());

        vector<Edge> edges(width * height * 4);
        size_t nEdges = 0;

        // Bounds on the thetas assigned to this group. Note that because
        // theta is periodic, these are defined such that the average
        // value is contained *within* the interval.
        { // limit scope of storage
            /* Previously all this was on the stack, but this is 1.2MB for 320x240 images
             * That's already a problem for OS X (default 512KB thread stack size),
             * could be a problem elsewhere for bigger images... so store on heap */
            vector<float> storage(width * height * 4);  // do all the memory in one big block, exception safe
            float *tmin = &storage[width * height * 0];
            float *tmax = &storage[width * height * 1];
            float *mmin = &storage[width * height * 2];
            float *mmax = &storage[width * height * 3];

            for (int y = 0; y + 1 < height; y++) {
                for (int x = 0; x + 1 < width; x++) {

                    float mag0 = fimMag.get(x, y);
                    if (mag0 < Edge::minMag)
                        continue;
                    mmax[y * width + x] = mag0;
                    mmin[y * width + x] = mag0;

                    float theta0 = fimTheta.get(x, y);
                    tmin[y * width + x] = theta0;
                    tmax[y * width + x] = theta0;

                    // Calculates then adds edges to 'vector<Edge> edges'
                    Edge::calcEdges(theta0, x, y, fimTheta, fimMag, edges, nEdges);

                    // XXX Would 8 connectivity help for rotated tags?
                    // Probably not much, so long as input filtering hasn't been disabled.
                }
            }

            edges.resize(nEdges);
            std::stable_sort(edges.begin(), edges.end());
            Edge::mergeEdges(edges, uf, tmin, tmax, mmin, mmax);
        }
/*
        struct timeval tpend1;
        gettimeofday(&tpend1, NULL);
        timeuse1 = 1000000*(tpend1.tv_sec - tpstart1.tv_sec)+tpend1.tv_usec-tpstart1.tv_usec;
        timeuse1/=1000.0;

	cout<<"timeuse1 is:"<<timeuse1<<endl;
 */

        struct timeval Step_four_extractTags;
        gettimeofday(&Step_four_extractTags, NULL);

        timeuse = 1000000 * (Step_four_extractTags.tv_sec - begin_extractTags.tv_sec) + Step_four_extractTags.tv_usec -
                  begin_extractTags.tv_usec;
        timeuse /= 1000.0;
        //if(timeuse >= 25 && index_up_down == 1)
        //return goodDetections;
        //================================================================
        // Step four: Loop over the pixels again, collecting statistics for each cluster.
        // We will soon fit lines (segments) to these points.

        map<int, vector<XYWeight> > clusters;
        for (int y = 0; y + 1 < fimSeg.getHeight(); y++) {
            for (int x = 0; x + 1 < fimSeg.getWidth(); x++) {
                if (uf.getSetSize(y * fimSeg.getWidth() + x) < Segment::minimumSegmentSize)
                    continue;

                int rep = (int) uf.getRepresentative(y * fimSeg.getWidth() + x);

                map<int, vector<XYWeight> >::iterator it = clusters.find(rep);
                if (it == clusters.end()) {
                    clusters[rep] = vector<XYWeight>();
                    it = clusters.find(rep);
                }
                vector<XYWeight> &points = it->second;
                points.push_back(XYWeight(x, y, fimMag.get(x, y)));
            }
        }

        struct timeval Step_five_extractTags;
        gettimeofday(&Step_five_extractTags, NULL);

        timeuse = 1000000 * (Step_five_extractTags.tv_sec - begin_extractTags.tv_sec) + Step_five_extractTags.tv_usec -
                  begin_extractTags.tv_usec;
        timeuse /= 1000.0;
        //if(timeuse >= 25 && index_up_down == 1)
        //return goodDetections;

        //================================================================
        // Step five: Loop over the clusters, fitting lines (which we call Segments).
        std::vector<Segment> segments; //used in Step six
        std::map<int, std::vector<XYWeight> >::const_iterator clustersItr;
        for (clustersItr = clusters.begin(); clustersItr != clusters.end(); clustersItr++) {
            std::vector<XYWeight> points = clustersItr->second;
            GLineSegment2D gseg = GLineSegment2D::lsqFitXYW(points);

            // filter short lines
            float length = MathUtil::distance2D(gseg.getP0(), gseg.getP1());
            if (length < Segment::minimumLineLength)
                continue;

            Segment seg;
            float dy = gseg.getP1().second - gseg.getP0().second;  //     /|
            float dx = gseg.getP1().first - gseg.getP0().first;    //    / |dy
                                                                   //   /__|
                                                                   //    dx
            float tmpTheta = std::atan2(dy, dx);

            seg.setTheta(tmpTheta);
            seg.setLength(length);

            // We add an extra semantic to segments: the vector
            // p1->p2 will have dark on the left, white on the right.
            // To do this, we'll look at every gradient and each one
            // will vote for which way they think the gradient should
            // go. This is way more retentive than necessary: we
            // could probably sample just one point!

            float flip = 0, noflip = 0;
            for (unsigned int i = 0; i < points.size(); i++) {
                XYWeight xyw = points[i];

                float theta = fimTheta.get((int) xyw.x, (int) xyw.y);   //what's the xyw represent for, where does it come from?
                float mag = fimMag.get((int) xyw.x, (int) xyw.y);

                // err *should* be +M_PI/2 for the correct winding, but if we
                // got the wrong winding, it'll be around -M_PI/2.
                float err = MathUtil::mod2pi(theta - seg.getTheta());

                if (err < 0)
                    noflip += mag;
                else
                    flip += mag;
            }

            if (flip > noflip) {
                float temp = seg.getTheta() + (float) M_PI;
                seg.setTheta(temp);
            }

            float dot = dx * std::cos(seg.getTheta()) + dy * std::sin(seg.getTheta());
            if (dot > 0) {
                seg.setX0(gseg.getP1().first);
                seg.setY0(gseg.getP1().second);
                seg.setX1(gseg.getP0().first);
                seg.setY1(gseg.getP0().second);
            } else {
                seg.setX0(gseg.getP0().first);
                seg.setY0(gseg.getP0().second);
                seg.setX1(gseg.getP1().first);
                seg.setY1(gseg.getP1().second);
            }

            segments.push_back(seg);
        }

#ifdef DEBUG_APRIL
#if 1
        {
            for (vector<Segment>::iterator it = segments.begin(); it != segments.end(); it++) {
                long int r = random();
                cv::line(image,
                         cv::Point2f(it->getX0(), it->getY0()),
                         cv::Point2f(it->getX1(), it->getY1()),
                         cv::Scalar(r % 0xff, (r % 0xff00) >> 8, (r % 0xff0000) >> 16, 0));
            }
        }
#endif
#endif

        struct timeval Step_six_extractTags;
        gettimeofday(&Step_six_extractTags, NULL);

        timeuse = 1000000 * (Step_six_extractTags.tv_sec - begin_extractTags.tv_sec) + Step_six_extractTags.tv_usec -
                  begin_extractTags.tv_usec;
        timeuse /= 1000.0;
        //if(timeuse >= 25 && index_up_down == 1)
        //return goodDetections;

        // Step six: For each segment, find segments that begin where this segment ends.
        // (We will chain segments together next...) The gridder accelerates the search by
        // building (essentially) a 2D hash table.
        Gridder<Segment> gridder(0, 0, width, height, 10);

        // add every segment to the hash table according to the position of the segment's
        // first point. Remember that the first point has a specific meaning due to our
        // left-hand rule above.
        for (unsigned int i = 0; i < segments.size(); i++) {
            gridder.add(segments[i].getX0(), segments[i].getY0(), &segments[i]);
        }

        // Now, find child segments that begin where each parent segment ends.
        for (unsigned i = 0; i < segments.size(); i++) {
            Segment &parentseg = segments[i];

            //compute length of the line segment
            GLine2D parentLine(std::pair<float, float>(parentseg.getX0(), parentseg.getY0()),
                               std::pair<float, float>(parentseg.getX1(), parentseg.getY1()));

            //find the child line based on the end point of parent line
            //start point of child line is in the range of end point of parent line
            Gridder<Segment>::iterator iter = gridder.find(parentseg.getX1(), parentseg.getY1(),
                                                           0.5f * parentseg.getLength());
            while (iter.hasNext()) {
                Segment &child = iter.next();
                //mod 2π
                if (MathUtil::mod2pi(child.getTheta() - parentseg.getTheta()) > 0) {
                    continue;
                }

                if(MathUtil::mod2pi(child.getTheta()-parentseg.getTheta())<(-PI/2-PI/18) ||
                   MathUtil::mod2pi(child.getTheta()-parentseg.getTheta())>(-PI/2+PI/18))
                    continue;

                // compute intersection of points
                GLine2D childLine(std::pair<float, float>(child.getX0(), child.getY0()),
                                  std::pair<float, float>(child.getX1(), child.getY1()));

                //calculate the intersection point of child line and parent line
                std::pair<float, float> p = parentLine.intersectionWith(childLine);
                if (p.first == -1) {
                    continue;
                }

                //calculate the distance between intersection point with parent(x1,y1)&child(x0,y0)
                float parentDist = MathUtil::distance2D(p,std::pair<float, float>(parentseg.getX1(), parentseg.getY1()));
                float childDist = MathUtil::distance2D(p, std::pair<float, float>(child.getX0(), child.getY0()));

                if (max(parentDist, childDist) > max(parentseg.getLength(),child.getLength())) {
                    continue;
                }

                // everything's OK, this child is a reasonable successor.
                parentseg.children.push_back(&child);
            }
        }

        struct timeval Step_seven_extractTags;
        gettimeofday(&Step_seven_extractTags, NULL);

        timeuse =1000000 * (Step_seven_extractTags.tv_sec - begin_extractTags.tv_sec) + Step_seven_extractTags.tv_usec -
                begin_extractTags.tv_usec;
        timeuse /= 1000.0;
        //if(timeuse >= 25 && index_up_down == 1)
        //return goodDetections;

        //================================================================
        // Step seven: Search all connected segments to see if any form a loop of length 4.
        // Add those to the quads list.
        vector<Quad> quads;

        vector<Segment *> tmp(5);
        for (unsigned int i = 0; i < segments.size(); i++) {
            tmp[0] = &segments[i];
            Quad::search(fimOrig, tmp, segments[i], 0, quads, opticalCenter);
        }

        if(quads.size()==0)
        {
            string tmp_path=preFixPath+"Rectfail/";
            if (access(tmp_path.c_str(), 0) == -1)	//如果文件夹不存在
                mkdir(tmp_path.c_str(),0777);
            string filename=tmp_path+"Rectfail"+to_string(N++)+".jpg";
            cv::imwrite(filename,image_src);
            cout<<"Rect dectect failed file save to: "<<filename<<endl;
        }

#ifdef DEBUG_APRIL
        {
            for (unsigned int j = 0; j < segments.size(); j++) {
                Segment segTmp = segments[j];
                cv::line(image_color, cv::Point2f(segTmp.getX0(), segTmp.getY0()),
                         cv::Point2f(segTmp.getX1(), segTmp.getY1()), cv::Scalar(0, 0, 255), 1);
                cv::namedWindow("debug_april_segment", cv::WINDOW_NORMAL);
                cv::imshow("debug_april_segment", image_color);
                cv::waitKey();
            }


            for (unsigned int qi = 0; qi < quads.size(); qi++) {
                Quad &quad = quads[qi];
                if(quad.observedPerimeter<PERIMETER_THRESHOLD)
                    continue;
                std::pair<float, float> p1 = quad.quadPoints[0];
                std::pair<float, float> p2 = quad.quadPoints[1];
                std::pair<float, float> p3 = quad.quadPoints[2];
                std::pair<float, float> p4 = quad.quadPoints[3];
                cv::line(image_color, cv::Point2f(p1.first, p1.second), cv::Point2f(p2.first, p2.second),
                         cv::Scalar(255, 0, 255, 0), 1);
                cv::line(image_color, cv::Point2f(p2.first, p2.second), cv::Point2f(p3.first, p3.second),
                         cv::Scalar(0, 255, 255, 0), 1);
                cv::line(image_color, cv::Point2f(p3.first, p3.second), cv::Point2f(p4.first, p4.second),
                         cv::Scalar(255, 255, 0, 0), 1);
                cv::line(image_color, cv::Point2f(p4.first, p4.second), cv::Point2f(p1.first, p1.second),
                         cv::Scalar(0, 255, 0, 0), 1);

                cv::namedWindow("debug_april_line_step7", cv::WINDOW_NORMAL);
                cv::imshow("debug_april_line_step7", image_color);
                cv::waitKey();

                p1 = quad.interpolate(-1, -1);
                p2 = quad.interpolate(-1, 1);
                p3 = quad.interpolate(1, 1);
                p4 = quad.interpolate(1, -1);
                cv::circle(image_color, cv::Point2f(p1.first, p1.second), 3, cv::Scalar(0, 255, 0, 0), 1);
                cv::circle(image_color, cv::Point2f(p2.first, p2.second), 3, cv::Scalar(0, 255, 0, 0), 1);
                cv::circle(image_color, cv::Point2f(p3.first, p3.second), 3, cv::Scalar(0, 255, 0, 0), 1);
                cv::circle(image_color, cv::Point2f(p4.first, p4.second), 3, cv::Scalar(0, 255, 0, 0), 1);
            }
//            cv::namedWindow("debug_april", cv::WINDOW_NORMAL);
//            cv::imshow("debug_april", image_color);
        }
#endif


        struct timeval Step_eight_extractTags;
        gettimeofday(&Step_eight_extractTags, NULL);

        timeuse =1000000 * (Step_eight_extractTags.tv_sec - begin_extractTags.tv_sec) + Step_eight_extractTags.tv_usec -
                begin_extractTags.tv_usec;
        timeuse /= 1000.0;
        //if(timeuse >= 25 && index_up_down == 1)
        //return goodDetections;
        //================================================================
        // Step eight. Decode the quads. For each quad, we first estimate a
        // threshold color to decide between 0 and 1. Then, we read off the
        // bits and see if they make sense.

        std::vector<TagDetection> detections;
        N++;
        bool saved=false;
        float max_perimeter=0;
        for(unsigned int pi=0; pi < quads.size();pi++)
        {
            Quad &q = quads[pi];
            if(q.observedPerimeter>max_perimeter)
                max_perimeter=q.observedPerimeter;
        }

        for (unsigned int qi = 0; qi < quads.size(); qi++) {
            Quad &quad = quads[qi];


            if (max_perimeter < PERIMETER_THRESHOLD)
            {
                string tmp_path=preFixPath+"Rectfail/";
                if (access(tmp_path.c_str(), 0) == -1)	//如果文件夹不存在
                    mkdir(tmp_path.c_str(),0777);
                string filename=tmp_path+"Rectfail"+to_string(N)+".jpg";
                if (!saved)
                {
                    cv::imwrite(filename,image_src);
                    cout<<"Rect dectect failed file save to: "<<filename<<endl;
                    saved=true;
                }
                continue;
            }

            // Find a threshold
            GrayModel blackModel, whiteModel;
            const int dd = 2 * thisTagFamily.blackBorder + thisTagFamily.dimension;

            for (int iy = -1; iy <= dd; iy++) {
                float y = (iy + 0.5f) / dd;
                for (int ix = -1; ix <= dd; ix++) {
                    float x = (ix + 0.5f) / dd;
                    std::pair<float, float> pxy = quad.interpolate01(x, y);
                    int irx = (int) (pxy.first + 0.5);
                    int iry = (int) (pxy.second + 0.5);
                    if (irx < 0 || irx >= width || iry < 0 || iry >= height)
                        continue;
                    float v = fim.get(irx, iry);
                    if (iy == -1 || iy == dd || ix == -1 || ix == dd)
                        whiteModel.addObservation(x, y, v);
                    else if (iy == 0 || iy == (dd - 1) || ix == 0 || ix == (dd - 1))
                        blackModel.addObservation(x, y, v);
                }
            }

            bool bad = false;
            unsigned long long tagCode = 0;
            for (int iy = thisTagFamily.dimension - 1; iy >= 0; iy--) {
                float y = (thisTagFamily.blackBorder + iy + 0.5f) / dd;
                for (int ix = 0; ix < thisTagFamily.dimension; ix++) {
                    float x = (thisTagFamily.blackBorder + ix + 0.5f) / dd;
                    std::pair<float, float> pxy = quad.interpolate01(x, y);
                    int irx = (int) (pxy.first + 0.5);
                    int iry = (int) (pxy.second + 0.5);
                    if (irx < 0 || irx >= width || iry < 0 || iry >= height) {
                        // cout << "*** bad:  irx=" << irx << "  iry=" << iry << endl;
                        bad = true;
                        continue;
                    }
                    float threshold = (blackModel.interpolate(x, y) + whiteModel.interpolate(x, y)) * 0.5f;
                    float v = fim.get(irx, iry);
                    tagCode = tagCode << 1;
                    if (v > threshold)
                        tagCode |= 1;
#ifdef DEBUG_APRIL
                    {
                        if (v > threshold)
                            cv::circle(image_color, cv::Point2f(irx, iry), 1, cv::Scalar(0, 0, 255, 0), 2);
                        else
                            cv::circle(image_color, cv::Point2f(irx, iry), 1, cv::Scalar(0, 255, 0, 0), 2);

                        cv::namedWindow("decode_quads_step8", cv::WINDOW_NORMAL);
                        cv::imshow("decode_quads_step8", image_color);
                    }
#endif
                }
            }

            if(bad)
            {
                string tmp_path=preFixPath+"Rectfail/";
                if (access(tmp_path.c_str(), 0) == -1)	//如果文件夹不存在
                    mkdir(tmp_path.c_str(),0777);
                string filename=tmp_path+"Rectfail"+to_string(N)+".jpg";
                if (!saved)
                {
                    cv::imwrite(filename,image_src);
                    cout<<"Bad dectect failed file save to: "<<filename<<endl;
                    saved=true;
                }
            }

            if (!bad) {
                TagDetection thisTagDetection;
                thisTagFamily.decode(thisTagDetection, tagCode);

                if(!thisTagDetection.good && thisTagDetection.hammingDistance<4)
                {
                    string ham_tmp_path=preFixPath+"Hamfail/";
                    if (access(ham_tmp_path.c_str(), 0) == -1)	//如果文件夹不存在
                        mkdir(ham_tmp_path.c_str(),0777);
                    string filename=ham_tmp_path+"Ham("+to_string(thisTagDetection.hammingDistance)+")fail_"+to_string(N)+".jpg";
                    if (!saved)
                    {
                        cv::imwrite(filename,image_src);
                        cout<<"Hamming failed file save to: "<<filename<<endl;
                        saved=true;
                    }
                }

                // compute the homography (and rotate it appropriately)
                thisTagDetection.homography = quad.homography.getH();
                thisTagDetection.hxy = quad.homography.getCXY();

                float c = std::cos(thisTagDetection.rotation * (float) M_PI / 2);
                float s = std::sin(thisTagDetection.rotation * (float) M_PI / 2);
                Eigen::Matrix3d R;
                R.setZero();
                R(0, 0) = R(1, 1) = c;
                R(0, 1) = -s;
                R(1, 0) = s;
                R(2, 2) = 1;
                Eigen::Matrix3d tmp;
                tmp = thisTagDetection.homography * R;
                thisTagDetection.homography = tmp;

                // Rotate points in detection according to decoded
                // orientation.  Thus the order of the points in the
                // detection object can be used to determine the
                // orientation of the target.
                std::pair<float, float> bottomLeft = thisTagDetection.interpolate(-1, -1);
                int bestRot = -1;
                float bestDist = FLT_MAX;
                for (int i = 0; i < 4; i++) {
                    float const dist = AprilTags::MathUtil::distance2D(bottomLeft, quad.quadPoints[i]);
                    if (dist < bestDist) {
                        bestDist = dist;
                        bestRot = i;
                    }
                }

                for (int i = 0; i < 4; i++)
                    thisTagDetection.p[i] = quad.quadPoints[(i + bestRot) % 4];

                if (thisTagDetection.good) {
                    thisTagDetection.cxy = quad.interpolate01(0.5f, 0.5f);
                    thisTagDetection.observedPerimeter = quad.observedPerimeter;
                    detections.push_back(thisTagDetection);
                }
            }
        }

#ifdef DEBUG_APRIL
        {
//    cv::namedWindow("debug_april_point",cv::WINDOW_NORMAL);
//    cv::imshow("debug_april_point", image_color);
        }
#endif

        struct timeval Step_nine_extractTags;
        gettimeofday(&Step_nine_extractTags, NULL);

        timeuse = 1000000 * (Step_nine_extractTags.tv_sec - begin_extractTags.tv_sec) + Step_nine_extractTags.tv_usec -
                  begin_extractTags.tv_usec;
        timeuse /= 1000.0;
        //if(timeuse >= 25 && index_up_down == 1)
        //return goodDetections;

        //================================================================
        //Step nine: Some quads may be detected more than once, due to
        //partial occlusion and our aggressive attempts to recover from
        //broken lines. When two quads (with the same id) overlap, we will
        //keep the one with the lowest error, and if the error is the same,
        //the one with the greatest observed perimeter.


        // NOTE: allow multiple non-overlapping detections of the same target.

        for (vector<TagDetection>::const_iterator it = detections.begin();
             it != detections.end(); it++) {
            const TagDetection &thisTagDetection = *it;

            bool newFeature = true;


            for (unsigned int odidx = 0; odidx < goodDetections.size(); odidx++) {
                TagDetection &otherTagDetection = goodDetections[odidx];

                if (thisTagDetection.id != otherTagDetection.id ||
                    !thisTagDetection.overlapsTooMuch(otherTagDetection))
                    continue;

                // There's a conflict.  We must pick one to keep.
                newFeature = false;

                // This detection is worse than the previous one... just don't use it.
                if (thisTagDetection.hammingDistance > otherTagDetection.hammingDistance)
                    continue;

                // Otherwise, keep the new one if it either has strictly *lower* error, or greater perimeter.
                if (thisTagDetection.hammingDistance < otherTagDetection.hammingDistance ||
                    thisTagDetection.observedPerimeter > otherTagDetection.observedPerimeter)
                    goodDetections[odidx] = thisTagDetection;
            }

            if (newFeature)
                goodDetections.push_back(thisTagDetection);

        }

        cout << "AprilTags: edges=" << nEdges << " clusters=" << clusters.size() << " segments=" << segments.size()
             << " quads=" << quads.size() << " detections=" << detections.size() << " unique tags="
             << goodDetections.size() << endl;

        return goodDetections;
    }

} // namespace

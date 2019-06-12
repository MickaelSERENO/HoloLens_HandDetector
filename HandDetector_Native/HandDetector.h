#pragma once

#include <cstdint>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <list>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#ifdef _OPENMP
#include <omp.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_HD(x, y) ((x) < (y) ? (y) : (x))
#define MIN_HD(x, y) ((x) > (y) ? (y) : (x))

namespace Sereno
{	
	/** Depth function typedef*/
	typedef uint16_t(*DepthFunc)(uint16_t i, uint16_t j, uint16_t width, uint8_t* data);

	/** Depth function working with 16 bits data */
    struct D16Func
    {
        /** Compute the depth 
         * \param i the x coordinate
         * \param j the y coordinate
         * \param width the data image width
         * \param data the image data */
        static uint16_t depthAt(uint16_t i, uint16_t j, uint16_t width, uint8_t* data)
        {
            return ((uint16_t*)(data))[i+j*width];
        }

        /** True data type */
        typedef uint16_t DepthType;

        /** The corresponding OpenCV Data */
        static const int CV_TYPE=CV_16UC1;
    };

    /** Depth function working with 8 bits data */
    struct D8Func
    {
        /** Compute the depth 
         * \param i the x coordinate
         * \param j the y coordinate
         * \param width the data image width
         * \param data the image data */
        static uint16_t depthAt(uint16_t i, uint16_t j, uint16_t width, uint8_t* data)
        {
            return (uint16_t)data[i+j*width];
        }

        /** True data type */
        typedef uint8_t DepthType;

        /** The corresponding OpenCV Data */
        static const int CV_TYPE=CV_8UC1;
    };

    struct Blob;

    /** Structure representing a Line */
    struct Line
    {
        uint16_t startX; /*!< The line starting point on the X axis*/
        uint16_t endX;   /*!< The line ending point on the X axis*/
        uint16_t y;      /*!< The y position */
        Blob* blob = NULL; /*!< The associated blob */
    };

    /** Structure representing a blob*/
    struct Blob
    {
        std::vector<Line*> lines; /*!< Line set which are included in this blob*/
        uint32_t area = 0;        /*!< The computed blob area*/
        uint16_t maxROI[2] = {0, 0}; /*!< The blob min Region of Interest*/
        uint16_t minROI[2] = {0, 0}; /*!< The blob max Region of Interest*/
    };

    /** Structure representing a finger*/
    struct Finger
    {
        uint16_t tipX; /*!< Finger tip X position in the original image*/
        uint16_t tipY; /*!< Finger tip Y position in the original image*/
    };

    /** Structure representing a hand*/
    struct Hand
    {
        std::vector<Finger> fingers; /*!< Finger detected*/
        uint16_t palmX; /*!< Palm X position*/
        uint16_t palmY; /*!< Palm Y position*/
    };

    /** Interface class permiting to detect the hands in a depth image*/
    class IHandDetection
    {
        public:
            /** Constructor
             * \param width the original image width
             * \param height the origin image height*/
            IHandDetection(uint16_t width, uint16_t height) : m_width(width), m_height(height)
            {}

            /** Destructor*/
            virtual ~IHandDetection(){}

            /** Update the hand position based on a new image
             * \param data the new image to evaluate*/
            virtual void updateDetection(uint8_t* data) = 0;

            /** Get the hand data
             * \return the hand data*/
            const std::vector<Hand>& getHands() const {return m_hands;}
        protected:
            uint16_t m_width;  /*!< The original image width*/
            uint16_t m_height; /*!< The original image height*/
            std::vector<Hand> m_hands; /*!< The last hands detected*/
    };

    template <typename TDepthFunc=D16Func>
    class HandDetection : public IHandDetection
    {
        public:
            /** \brief  Constructor
             * \param width the images width
             * \param height the images height
             * \param maxRange the maximum range (z depth) of the hand position (e.g., 1000 for 1 meter)
			 * \param minRange the minimum range (z depth) of the hand position (e.g., 200 for 20 cm)
             * \param maxDelta the maximum delta beteen the pixels neighboor in order to separate blobs
             * \param minArea the minimum hand area to look at (e.g., 500) 
			 * \param maxHandLength the maximum hand length in pixel along the major axis*/
            HandDetection(uint16_t width, uint16_t height, uint16_t minRange, uint16_t maxRange, uint16_t maxDelta, uint16_t minArea, uint16_t maxHandLength) : IHandDetection(width, height), m_minRange(minRange), m_maxRange(maxRange), m_maxDelta(maxDelta), m_minArea(minArea), m_maxHandLength(maxHandLength)
            {}

            void updateDetection(uint8_t* data)
            {
                std::vector<Line*> lines;
                std::list<Blob*> blobs;
                uint16_t idLineN1 = 0;
                uint16_t idLineN2 = 0;

                //Detect lines
                for(uint16_t j = 0; j < m_height; j++)
                {
                    uint16_t lastDepth = 0, currentDepth = 0;
                    idLineN2 = (uint16_t)lines.size();
                    for(uint16_t i = 0; i < m_width; i++)
                    {
                        //Skip useless data
                        while(i < m_width)
                        {
                            currentDepth = TDepthFunc::depthAt(i, j, m_width, data);
                            if(currentDepth <= m_maxRange && currentDepth != 0 && currentDepth >= m_minRange)
                                break;
                            i++;
                        }
                        if(i >= m_width)
                            break;

                        lastDepth = currentDepth;

                        //Start a line
                        Line* line = new Line();
                        line->startX = i;
                        line->y = j;

                        lines.push_back(line);

                        //Advance in this line
                        if(i+1 < m_width)
                        {
                            i++;
                            while(i < m_width)
                            {
                                currentDepth = TDepthFunc::depthAt(i, j, m_width, data);
                                if(currentDepth > m_maxRange || currentDepth == 0 || currentDepth < m_minRange || std::abs(currentDepth-lastDepth) > m_maxDelta)
                                    break;
                                lastDepth = currentDepth;
                                i++;
                            }
                            i--;
                        }
                        line->endX = i;
                    }

                    //Update the blobs
                    updateBlob(blobs, lines, idLineN1, idLineN2);
                    idLineN1 = idLineN2;
                }
                //Insert the last blobs
                updateBlob(blobs, lines, idLineN1, (uint16_t)lines.size());

                //Discard blobs with not enough area
                for(auto it = blobs.begin(); it != blobs.end();)
                {
                    if((*it)->area < m_minArea)
                    {
                        Blob* b = (*it);
                        it = blobs.erase(it);
                        delete b;
                    }
                    else
                        it++;
                }

                //Compute the hands position
                computeHandDatas(blobs);

                //Free everything
                for(auto it = blobs.begin(); it != blobs.end(); it++)
                    delete *it;
                for(auto it = lines.begin(); it != lines.end(); it++)
                    delete *it;
            }

        private:
            /** Update the blob data when a new image row has been processed
             * \param blobs[out] the blobs to update
             * \param lines[in] the lines data of ALL the image
             * \param idLineN1[in] the index where the line N-1 starts in the lines array. idLineN2-idLineN1 == number of lines in the row N-1 
             * \param idLineN2[in] the index where the line N start in the lines array. lines.size() - idLineN2 == number of lines in the row N-2*/
            void updateBlob(std::list<Blob*>& blobs, const std::vector<Line*>& lines, uint16_t idLineN1, uint16_t idLineN2)
            {
                //No lines added
                if(idLineN2 == idLineN1)
                    return;

                for(uint16_t id1 = idLineN1; id1 < idLineN2; id1++)
                {
                    Line* lineN1 = lines[id1];

                    //Create a blob if needed
                    if(lineN1->blob == NULL)
                    {
                        Blob* blob = new Blob();
                        blob->lines.push_back(lineN1);
                        blob->area = lineN1->endX - lineN1->startX + 1;
                        blob->maxROI[0] = lineN1->endX;
                        blob->minROI[0] = lineN1->startX;
                        blob->minROI[1] = blob->maxROI[1] = lineN1->y;

                        lineN1->blob = blob;

                        blobs.push_back(blob);
                    }

                    //Test connectivity per line
                    for(uint16_t id2 = idLineN2; id2 < lines.size(); id2++)
                    {
                        Line* lineN2 = lines[id2];
                        if(lineN1->endX > lineN2->startX && lineN1->startX < lineN2->endX)
                        {
                            //Add this line to the blob
                            if(lineN2->blob == NULL)
                            {
                                Blob* blob = lineN1->blob;
                                blob->lines.push_back(lineN2);
                                blob->area += lineN2->endX - lineN2->startX + 1;
                                uint16_t maxROI[] = {lineN2->endX,   lineN2->y};
                                uint16_t minROI[] = {lineN2->startX, lineN2->y};

                                for(uint8_t k = 0; k < 2; k++)
                                {
                                    blob->maxROI[k] = MAX_HD(maxROI[k], blob->maxROI[k]);
                                    blob->minROI[k] = MIN_HD(minROI[k], blob->minROI[k]);
                                }

                                lineN2->blob = blob;
                            }

                            //Merge blobs
                            else if(lineN2->blob != lineN1->blob)
                            {
                                //Use lineN1->blob as the current blob
                                Blob* blob = lineN1->blob;

                                //Update blobs array and blobs assignment
                                for(auto it = blobs.begin(); it != blobs.end(); it++)
                                {
                                    if(*it == lineN2->blob)
                                    {
                                        //Update area and ROI
                                        for(uint8_t k = 0; k < 2; k++)
                                        {
                                            blob->minROI[k] = MIN_HD(blob->minROI[k], lineN2->blob->minROI[k]);
                                            blob->maxROI[k] = MAX_HD(blob->maxROI[k], lineN2->blob->maxROI[k]);
                                        }
                                        blob->area += lineN2->blob->area;

                                        //Update assignment of each lines
                                        for(Line* l : lineN2->blob->lines)
                                        {
                                            blob->lines.push_back(l);
                                            l->blob = blob;
                                        }
                                        Blob* bToErase = *it;
                                        blobs.erase(it);
                                        delete bToErase;
                                        break;
                                    }
                                }
                            } 
                        }
                    }
                }
            }

            /** Update the hands data via the blobs computed
             * \param blobs the blobs computed*/
            void computeHandDatas(const std::list<Blob*>& blobs)
            {
                m_hands.clear();

                /* Each blob can be a hand... */
                for(Blob* b : blobs)
                {
                    if(b->lines.size() > 1)
                    {
                        cv::Mat img = cv::Mat::zeros(cv::Size(b->maxROI[0]-b->minROI[0]+1, b->maxROI[1]-b->minROI[1]+1), CV_8UC1);

						int32_t lineSize = (int32_t)b->lines.size();
#ifdef _OPENMP
                        #pragma omp parallel for
#endif
                        for(int32_t i = 0; i < lineSize; i++)
                        {
                            Line* l = b->lines[i];
                            for(uint32_t j = l->startX; j < l->endX; j++)
                            {
                                img.at<uchar>(l->y-b->minROI[1], j-b->minROI[0]) = 255;
                                img.at<uchar>(l->y-b->minROI[1], j-b->minROI[0]) = 255;
                            }
                        }

                        //Find the contour
                        std::vector<std::vector<cv::Point>> contours;
                        std::vector<cv::Vec4i> hierarchy;

                        cv::findContours(img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

                        int32_t maxContourID = 0;

                        //Compute the convex hull
                        std::vector<int> hull;
                        cv::convexHull(contours[maxContourID], hull);

                        //If the convex hull is computed (i.e., at least 3 points)
                        if(hull.size() > 3)
                        {
                            bool firstFingerComputed = false;
                            cv::Point firstFinger(img.size[1]-1, img.size[0]-1);
                            cv::Rect handROI(0, 0, img.cols, img.rows);
                            cv::Point wristPos(0, 0);

                            //Delete the forearm along the vertical axis
                            if(img.size[1] < img.size[0])
                                firstFingerComputed = getWristROI(img, contours[maxContourID], hull, firstFinger, handROI, wristPos);
                            else
                            {
                                //Determine most upper part
                                int minY = img.size[0];
                                int minX = 0;

                                for(int i = 0; i < img.size[1]; i++)
                                    for(int j = 0; j < minY; j++)
                                        if(img.at<uchar>(j, i) == 255 && j < minY)
                                        {
                                            minY = j;
                                            minX = i;
                                            break;
                                        }

                                int cw = -1;

                                //If most upper part is on the left side of the image...
                                if(minX < img.size[1]/2)
                                    cw = 1;

                                //Rotate image
                                cv::Mat rotImage;
                                std::vector<cv::Point> rotContour = contours[maxContourID];

                                //Transpose and flip
                                transpose(img, rotImage);  
                                if(cw == 1)
                                {
                                    for(auto& p : rotContour)
                                    {
                                        int old = p.y;
                                        p.y = p.x;
                                        p.x = old;

                                        p.x = rotImage.cols - 1 - p.x;
                                    }
                                    flip(rotImage, rotImage, 1);
                                }
                                else
                                {
                                    for(auto& p : rotContour)
                                    {
                                        int old = p.y;
                                        p.y = p.x;
                                        p.x = old;

                                        p.y = rotImage.rows - 1 - p.y;
                                    }
                                    flip(rotImage, rotImage, 0);
                                }

                                firstFingerComputed = getWristROI(rotImage, rotContour, hull, firstFinger, handROI, wristPos);

                                //Apply the inverse transformation

                                //Hand
                                int roiHeight = handROI.height;
                                handROI.height = handROI.width;
                                handROI.width  = roiHeight;
                                if(cw == -1)
                                    handROI.x = img.cols-handROI.width;
                                
                                //Finger
                                if(firstFingerComputed)
                                {
                                    int firstFingerX = firstFinger.x;
                                    firstFinger.x = firstFinger.y;
                                    firstFinger.y = firstFingerX;
                                    if(cw == -1)
                                        firstFinger.x = img.size[1]-firstFinger.x;
                                    else
                                        firstFinger.y = img.size[0]-firstFinger.y;

                                }

                                //Wrist
                                int wristPosX = wristPos.x;
                                wristPos.x = wristPos.y;
                                wristPos.y = wristPosX;
                                if(cw == -1)
                                    wristPos.x = img.size[1]-wristPos.x;
                                else
                                    wristPos.y = img.size[0]-wristPos.y;
                            }

                            //Now we can add this new detected hand
                            cv::Mat handImg = img(handROI);
                            wristPos.x += b->minROI[0] + handROI.x;
                            wristPos.y += b->minROI[1] + handROI.y;
                            if(!addNewHand(b, handImg, handROI, contours[maxContourID], hull, (firstFingerComputed ? &firstFinger : NULL), wristPos))
                                continue;
                        }
                    }
                }
            }

            /**
             * \brief  Add a new hand based on the computed data done in computeHandDatas
             * \param b the current blob
             * \param handImg the hand image treated. 0 == no hand pixel, 255 == hand pixel
             * \param handROI the hand ROI in the blob (this is computed to remove the wrist from the arm)
             * \param contour the hand contour
             * \param hull the hand hull
             * \param firstFinger the first finger found. NULL if no first finger
             * \param wristPos the detected wrist position
             * \return true if the hand was fully added, false otherwise. If false, it means that the hand was not detected as a hand */
            bool addNewHand(const Blob* b, const cv::Mat& handImg, cv::Rect& handROI, const std::vector<cv::Point>& contour, const std::vector<int>& hull, const cv::Point* firstFinger, const cv::Point& wristPos)
            {
                Hand hand;

                //Compute the convexity defects
                std::vector<cv::Vec4i> defects;
                cv::convexityDefects(contour, hull, defects);

                //Compute the distance transform to find the Palm position
                cv::Mat distance;
                cv::distanceTransform(handImg, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);

                //Now determine the hand position
                float maxPalmDist = distance.at<float>(0, 0);
                hand.palmX = 0;
                hand.palmY = 0;
#ifdef _OPENMP
                #pragma omp parallel
                {
                    int ompPalmX = 0;
                    int ompPalmY = 0;
                    float ompMaxPalmDist = distance.at<float>(0, 0);
                    #pragma omp for
                    for(int i = 0; i < distance.rows; i++) 
                    {
                        for (int j = 0; j < distance.cols; j++) 
                        {
                            float dist = distance.at<float>(i, j);
                            if(ompMaxPalmDist < dist) 
                            {
                                ompPalmX = j; 
                                ompPalmY = i;
                                ompMaxPalmDist = dist;
                            }
                        }
                    }

                    #pragma omp critical
                    {
                        if(ompMaxPalmDist > maxPalmDist)
                        {
                            maxPalmDist = ompMaxPalmDist;
                            hand.palmX = ompPalmX;
                            hand.palmY = ompPalmY;
                        }
                    }
                }
#else
                for(int i = 0; i < distance.rows; i++) 
                {
                    for (int j = 0; j < distance.cols; j++) 
                    {
                        float dist = distance.at<float>(i, j);
                        if(maxPalmDist < dist) 
                        {
                            hand.palmX = j; 
                            hand.palmY = i;
                            maxPalmDist = dist;
                        }
                    }
                }
#endif

                hand.palmX += b->minROI[0]+handROI.x;
                hand.palmY += b->minROI[1]+handROI.y;

                //Compute the finger position
                std::vector<cv::Vec4i> validDefects;

                for(const cv::Vec4i& v : defects)
                {
                    float depth = v[3] / 256.0f;
                    if(depth > 20.0f) //  filter defects by depth, e.g more than 20
                    {
                        int startID = v[0]; cv::Point ptStart(contour[startID]);
                        int endID   = v[1]; cv::Point ptEnd(contour[endID]);
                        int farID   = v[2]; cv::Point ptFar(contour[farID]);

                        if(ptStart.y < handImg.size[0] &&
                           ptEnd.y   < handImg.size[0] &&
                           ptFar.y   < handImg.size[0])
                        {
                            //Filter by angle
                            float vec1X, vec2X, vec1Y, vec2Y;
                            vec1X = (float)(ptFar.x - ptStart.x);
                            vec1Y = (float)(ptFar.y - ptStart.y);
                            vec2X = (float)(ptFar.x - ptEnd.x);
                            vec2Y = (float)(ptFar.y - ptEnd.y);
                            float mag1 = (float)sqrt(vec1X*vec1X + vec1Y*vec1Y);
                            float mag2 = (float)sqrt(vec2X*vec2X + vec2Y*vec2Y);

                            vec1X/=mag1;
                            vec1Y/=mag1;
                            vec2X/=mag2;
                            vec1Y/=mag2;

                            float angle = acos(vec1X*vec2X + vec1Y*vec2Y);
                            if(angle > 180)
                                angle -= 180;
                            if(angle < M_PI/2.0f)
                                validDefects.push_back(v);
                        }
                    }
                }

				if (validDefects.size() <= 5)
				{
					if (validDefects.size() > 0)
					{
						int startID = validDefects[0][0]; cv::Point ptStart(contour[startID]);
						Finger finger;
						finger.tipX = ptStart.x + b->minROI[0] + handROI.x;
						finger.tipY = ptStart.y + b->minROI[1] + handROI.y;

						if (!fingerPointsTop(b, handROI, hand, finger, handImg, wristPos))
							return false;
						hand.fingers.push_back(finger);
						for (const auto& v : validDefects)
						{
							int endID = v[1]; cv::Point ptEnd(contour[endID]);
							finger.tipX = ptEnd.x + b->minROI[0] + handROI.x;
							finger.tipY = ptEnd.y + b->minROI[1] + handROI.y;
							if (!fingerPointsTop(b, handROI, hand, finger, handImg, wristPos))
								return false;
							hand.fingers.push_back(finger);
						}
					}

					else if (firstFinger)
					{
						Finger finger;
						finger.tipX = firstFinger->x + b->minROI[0];
						finger.tipY = firstFinger->y + b->minROI[1];

						if (finger.tipX*finger.tipX + finger.tipY*finger.tipY >= m_minFirstSqDist)
						{
							if (!fingerPointsTop(b, handROI, hand, finger, handImg, wristPos))
								return false;
							hand.fingers.push_back(finger);
						}
					}

					m_hands.push_back(hand);
					return true;
				}
				return false;
            }

            /**
             * \brief  Detect the wrist ROI in a head binary image. It detects, from the most left (and right) hull to the bottom of the image where is the line with the lowest number of pixels
             *
             * \param img the original image. 0 == no hand, 255 == hand pixel
             * \param contour the contour computed
             * \param hull the convex hull computed
             * \param firstFinger[out] where is the detected first finger position? This is the topest pixel position
             * \param roi[out] the ROI computed
             * \param wristPos[out] the wrist position
             *
             * \return   is the first finger computed? */
            bool getWristROI(const cv::Mat& img, const std::vector<cv::Point>& contour, const std::vector<int>& hull, cv::Point& firstFinger, cv::Rect& roi, cv::Point& wristPos)
            {
                roi = cv::Rect(0, 0, img.size[1]-1, img.size[0]-1);
                bool firstFingerComputed = true;

                //Check for the left and right most hull point
                int minHullX   = img.size[1];
                int minHullY   = img.size[0];
                int maxHullY   = 0;
                int maxHullX   = 0;

                for(uint32_t i = 0; i < hull.size(); i++)
                {
                    if((contour[hull[i]].x < minHullX) || 
                       (contour[hull[i]].x == minHullX && contour[hull[i]].y > minHullY))
                    {
                        minHullX = contour[hull[i]].x; 
                        minHullY = contour[hull[i]].y;
                    }

                    if((contour[hull[i]].x > maxHullX) || 
                       (contour[hull[i]].x == maxHullX && contour[hull[i]].y > maxHullY))
                    {
                        maxHullX = contour[hull[i]].x; 
                        maxHullY = contour[hull[i]].y;
                    }

                    //Catch the first finger
                    if(firstFinger.y > contour[hull[i]].y)
                    {
                        firstFinger.x = contour[hull[i]].x;
                        firstFinger.y = contour[hull[i]].y;
                    }
                }

                //Take the most upper point
                int hullY = MIN_HD(minHullY, maxHullY);

				//Truncate along the maximum length
				int maxLengthSq = m_maxHandLength*m_maxHandLength/(img.size[0]*img.size[0] + img.size[1]*img.size[1]) * img.size[0]*img.size[0];
				if(hullY*hullY > maxLengthSq)
					hullY = (int)sqrt(maxLengthSq);

                if(hullY < img.size[1])
                {
                    //Create a vertical graph
                    size_t verticalGraphSize = MIN_HD(2*hullY, img.size[0]-hullY);
                    uint16_t* verticalGraph = (uint16_t*)calloc(verticalGraphSize, sizeof(uint16_t));

                    for(int j = 0; j < img.size[1]; j++)
                    {
                        for(size_t i = 0; i < verticalGraphSize; i++)
                        {
                            if(img.at<uchar>(i+hullY, j) == 255)
                                verticalGraph[i]++;                                                    
                        }
                    }

                    //Delete from the minimum row
                    int rowWrist = -1;
                    int minRowWristValue = img.size[1];
                    for(size_t i = 0; i < verticalGraphSize; i++)
                    {
                        if(minRowWristValue > verticalGraph[i])
                        {
                            minRowWristValue = verticalGraph[i];
                            rowWrist = i;
                        }
                    }

                    free(verticalGraph);

                    //Define the new ROI
                    roi = cv::Rect(0, 0, img.size[1]-1, rowWrist+hullY);

                    //Get hand wrist position
                    int minPosWristX = 0;
                    int maxPosWristX = 0;
                    for(int i = 0; i < img.cols; i++)
                        if(img.at<uint8_t>(rowWrist+hullY-1, i) == 255)
                        {
                            minPosWristX = i;
                            break;
                        }

                    for(int i = minPosWristX; i < img.cols; i++)
                        if(img.at<uint8_t>(rowWrist+hullY-1, i) == 255)
                            maxPosWristX = i;

                    wristPos.x = (maxPosWristX+minPosWristX)/2;
                    wristPos.y = rowWrist+hullY;
                }

                return firstFingerComputed;
            }

            /* \brief  Tell if a finger is pointing top or not. If we found a finger not pointing top, the object is not a hand
             * \param hand the hand
             * \param finger the finger to evaluate, part of the hand
             * \param handImg the hand image. We use it in order to get the hand direction
             * \return  true if the finger points top, false otherwise */
            bool fingerPointsTop(const Blob* b, const cv::Rect& handROI, const Hand& hand, const Finger& finger, const cv::Mat& handImg, const cv::Point& wristPos)
            {
                if(finger.tipY >= wristPos.y)
                    return false;
                int fX = finger.tipX - wristPos.x;
                int fY = finger.tipY - wristPos.y;

                return (fX * (finger.tipX - hand.palmX) + 
                        fY * (finger.tipY - hand.palmY)) >= 0;
            }

			uint16_t m_maxHandLength;
			uint16_t m_minRange;
            uint16_t m_maxRange;
            uint16_t m_maxDelta;
            uint16_t m_minArea;
            uint16_t m_minFirstSqDist=20;
    };
}
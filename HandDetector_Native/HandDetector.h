#pragma once

#include <cstdint>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <list>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_HD(x, y) ((x) < (y) ? (y) : (x))
#define MIN_HD(x, y) ((x) > (y) ? (y) : (x))

namespace Sereno
{
	typedef uint16_t(*DepthFunc)(uint16_t i, uint16_t j, uint16_t width, uint8_t* data);

    /** Depth function working with 16 bits data */
    struct D16Func
    {
        /** Compute the depth 
         * \param i the x coordinate
         * \param j the y coordinate
         * \param width the data image width
         * \param data the image data */
        static uint16_t computeDepth(uint16_t i, uint16_t j, uint16_t width, uint8_t* data)
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
		static uint16_t computeDepth(uint16_t i, uint16_t j, uint16_t width, uint8_t* data)
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
    class IHandDetector
    {
        public:
            /** Constructor
             * \param width the original image width
             * \param height the origin image height*/
            IHandDetector(uint16_t width, uint16_t height) : m_width(width), m_height(height)
            {}

            /** Destructor*/
            virtual ~IHandDetector(){}

            /** Update the hand position based on a new image
             * \param data the new image to evaluate*/
            virtual void updateDetector(uint8_t* data) = 0;

            /** Get the hand data
             * \return the hand data*/
            const std::vector<Hand>& getHands() const {return m_hands;}
        protected:
            uint16_t m_width;  /*!< The original image width*/
            uint16_t m_height; /*!< The original image height*/
            std::vector<Hand> m_hands; /*!< The last hands detected*/
    };

    template <typename T_DepthFunc=D16Func>
    class HandDetector : public IHandDetector
    {
        public:
            HandDetector(uint16_t width, uint16_t height, uint16_t maxRange, uint16_t maxDelta, uint16_t minArea) : IHandDetector(width, height), m_maxRange(maxRange), m_maxDelta(maxDelta), m_minArea(minArea)
            {}

            void updateDetector(uint8_t* data)
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
                            currentDepth = T_DepthFunc::computeDepth(i, j, m_width, data);
                            if(currentDepth <= m_maxRange && currentDepth != 0)
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
                                currentDepth = T_DepthFunc::computeDepth(i, j, m_width, data);
                                if(currentDepth > m_maxRange || currentDepth == 0 || std::abs(currentDepth-lastDepth) > m_maxDelta)
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
#ifdef OUTPUT_HD
                uint32_t idImage = 0;
#endif
                m_hands.clear();

                /* Each blob can be a hand... */
                for(Blob* b : blobs)
                {
                    if(b->lines.size() > 1)
                    {
                        cv::Mat img = cv::Mat::zeros(cv::Size(b->maxROI[0]-b->minROI[0]+1, b->maxROI[1]-b->minROI[1]+1), CV_8UC1);

                        for(Line* l : b->lines)
                        {
                            for(uint32_t j = l->startX; j < l->endX; j++)
                            {
                                img.at<uchar>(l->y-b->minROI[1], j-b->minROI[0]) = 255;
                                img.at<uchar>(l->y-b->minROI[1], j-b->minROI[0]) = 255;
                            }
                        }

                        //Find the contour
                        std::vector<std::vector<cv::Point>> contours;
                        std::vector<cv::Vec4i> hierarchy;

                        cv::findContours(img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

                        int32_t maxContourID = -1;
                        double  maxContour   = 0;
                        for(int32_t i = 0; i < (int)contours.size(); i++)
                        {
                            double area = cv::contourArea(contours[i]);
                            if(area > maxContour)
                            {
                                maxContourID = i;
                                maxContour = area;
                            }
                        }

                        //If a Contour if found
                        if(maxContour > 0)
                        {
                            //Compute the convex hull
                            std::vector<int> hull;
                            cv::convexHull(contours[maxContourID], hull);

                            //If the convex hull is computed (i.e., at least 3 points)
                            if(hull.size() > 3)
                            {
                                m_hands.emplace_back();
                                Hand& hand = m_hands.back();

                                //Compute the convexity defects
                                std::vector<cv::Vec4i> defects;
                                cv::convexityDefects(contours[maxContourID], hull, defects);

                                //Compute the distance transform to find the Palm position
                                cv::Mat distance;
                                cv::distanceTransform(img, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);

                                //Now determine the hand position
                                float maxPalmDist = 0;
                                for (int i = 0; i < distance.rows; i++) 
                                {
                                    for (int j = 0; j < distance.cols; j++) 
                                    {
                                        float dist = distance.at<float>(i, j);
                                        if (maxPalmDist < dist) 
                                        {
                                            hand.palmX = j; 
                                            hand.palmY = i;
                                            maxPalmDist = dist;
                                        }
                                    }
                                }

#ifdef OUTPUT_HD
                                cv::Mat contourImg(img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
                                char fileName[1024];
                                sprintf(fileName, "File%03d.png", idImage);
                                cv::imwrite(fileName, img);

                                cv::drawContours(contourImg, contours, maxContourID, cv::Scalar(255, 0, 0));
                                cv::circle(contourImg, cv::Point(hand.palmX, hand.palmY), 4, cv::Scalar(255, 255, 255), 2);
#endif
                                hand.palmX += b->minROI[0];
                                hand.palmY += b->minROI[1];

                                //Compute the finger position
                                std::vector<cv::Vec4i> validDefects;

                                for(const cv::Vec4i& v : defects)
                                {
                                    float depth = v[3] / 256.0f;
                                    if (depth > 20.0f) //  filter defects by depth, e.g more than 20
                                    {
                                        int startID = v[0]; cv::Point ptStart(contours[maxContourID][startID]);
                                        int endID   = v[1]; cv::Point ptEnd(contours[maxContourID][endID]);
                                        int farID   = v[2]; cv::Point ptFar(contours[maxContourID][farID]);

                                        //Filter by angle
                                        float vec1X, vec2X, vec1Y, vec2Y;
                                        vec1X = (float)(ptFar.x - ptStart.x);
                                        vec1Y = (float)(ptFar.y - ptStart.y);
                                        vec2X = (float)(ptFar.x - ptEnd.x);
                                        vec2Y = (float)(ptFar.y - ptEnd.y);
                                        float mag1 = sqrt(vec1X*vec1X + vec1Y*vec1Y);
                                        float mag2 = sqrt(vec2X*vec2X + vec2Y*vec2Y);

                                        vec1X/=mag1;
                                        vec1Y/=mag1;
                                        vec2X/=mag2;
                                        vec1Y/=mag2;

                                        float angle = acos(vec1X*vec2X + vec1Y*vec2Y);
                                        if(angle > 180)
                                            angle -= 180;
                                        if(angle < M_PI/2.0f)
                                        {
                                            validDefects.push_back(v);
#ifdef OUTPUT_HD
                                            cv::line(contourImg, ptStart, ptEnd, cv::Scalar(0, 255, 0), 1);
                                            cv::line(contourImg, ptStart, ptFar, cv::Scalar(0, 255, 0), 1);
                                            cv::line(contourImg, ptEnd, ptFar,   cv::Scalar(0, 255, 0), 1);
                                            cv::circle(contourImg, ptFar, 4,     cv::Scalar(0, 255, 0), 2);
                                            cv::circle(contourImg, ptStart, 4, cv::Scalar(0, 0, 255), 2);
#endif
                                        }
                                    }
                                }

                                if(validDefects.size() > 0)
                                {
                                    int startID = validDefects[0][0]; cv::Point ptStart(contours[maxContourID][startID]);
                                    Finger finger;
                                    finger.tipX = ptStart.x + b->minROI[0];
                                    finger.tipY = ptStart.y + b->minROI[1];
                                    hand.fingers.push_back(finger);
                                    for(const auto& v : validDefects)
                                    {
                                        int endID   = v[1]; cv::Point ptEnd(contours[maxContourID][endID]);
                                        finger.tipX = ptEnd.x + b->minROI[0];
                                        finger.tipY = ptEnd.y + b->minROI[1];
                                        hand.fingers.push_back(finger);
                                    }
                                }
#ifdef OUTPUT_HD
                                sprintf(fileName, "Contour%03d.png", idImage);
                                cv::imwrite(fileName, contourImg);
#endif
                            }
                        }

#ifdef OUTPUT_HD
                        idImage++;
#endif
                    }
                }
            }

            uint16_t m_maxRange;
            uint16_t m_maxDelta;
            uint16_t m_minArea;
    };
}
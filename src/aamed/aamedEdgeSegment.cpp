#include "aamed/AAMED.h"

#include <utility>

  void AAMED::convert_edge_image(const cv::Mat imgInput, cv::Mat &imageEdge)
  {
    // TODO: remove useless clone or copy operation
    cv::Mat imgThreshold;

    // use adaptiveThreshold() method can overcome some uneven lighting problem
    // TODO: can we use the image entropy to determine the constant C parameter? (-10)
    cv::adaptiveThreshold(imgInput, imgThreshold, 255., cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 13, -10);

    // TODO: maybe dilate operation can avoid the inner block stick to the outer border situation?
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    cv::dilate(imgThreshold, imgThreshold, kernel, cv::Point(-1, -1), 1);

    // findContours() method will generate more continuous edge segments than Canny() method
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(imgThreshold, contours, cv::noArray(), cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    
    // remove too small contours
    int minContourSize = 30;
    for (int i = 0; i < contours.size(); ++i) {
      if (contours[i].size() > minContourSize) {
        cv::drawContours(imageEdge, contours, i, cv::Scalar(255, 255, 255), 1);
      }
    }

#if DEBUG    
    cv::imshow("imageThres", imgThreshold);
    cv::imshow("dilate", imgThreshold);
    cv::imshow("imageEdge", imageEdge);
    cv::waitKey();
#endif
  }

  


  /**
   * @brief AAMED trace FindContour 
   * Move left from the given pixel to the end and also do the other direction
   * connect both to a whole segments
   * @param Wise 
   * @param antiWise 
   * @param Edge 
   * @param x 
   * @param y
   * @param edgeSegments
   * @param edgeSegmentsForCluster 
   */
  void AAMED::trace_pixel(const int Wise[8][2], const int antiWise[8][2], 
                            uchar *Edge, int x, int y,
                            std::vector<std::vector<cv::Point>> &edgeSegments,
                            std::vector<EdgeSegmentYXParam> &edgeSegmentsForCluster)
  {
    bool isEnd;
    int find_x, find_y;
    int move_x = x, move_y = y;
    std::vector<cv::Point> oneContour; 
    std::vector<cv::Point> oneContourOpp; 
    oneContour.push_back(cv::Point(x, y));
    int idxdMove = dIDX(x, y), idxiMove = iIDX(x, y), idxdFind, idxiFind;
    data[idxdMove].set_first();
    while (1)
    {
      isEnd = true;
      idxiMove = iIDX(move_x, move_y);
      for (int i = 0; i < 8; i++)
      {
        find_x = move_x + Wise[i][0];
        find_y = move_y + Wise[i][1];
        idxiFind = iIDX(find_x, find_y);
        if (Edge[idxiFind])
        {
          Edge[idxiFind] = 0;
          isEnd = false;
          idxdMove = dIDX(move_x, move_y);
          idxdFind = dIDX(find_x, find_y);
          (data + idxdMove)->next_is(data + idxdFind);
          move_x = find_x; move_y = find_y;
          oneContour.push_back(cv::Point(move_x, move_y));
          break;
        }
      }
      if (isEnd)
      {
        idxdMove = dIDX(move_x, move_y);
        (data + idxdMove)->next_is(NULL);
        break;
      }
    }
    move_x = oneContour[0].x; move_y = oneContour[0].y;
    while (1)
    {
      isEnd = true;
      idxiMove = iIDX(move_x, move_y);
      for (int i = 0; i < 8; i++)
      {
        find_x = move_x + antiWise[i][0];
        find_y = move_y + antiWise[i][1];
        idxiFind = iIDX(find_x, find_y);
        if (Edge[idxiFind])
        {
          Edge[idxiFind] = 0;
          isEnd = false;
          idxdMove = dIDX(move_x, move_y);
          idxdFind = dIDX(find_x, find_y);
          (data + idxdMove)->last_is(data + idxdFind);
          move_x = find_x; move_y = find_y;
          oneContourOpp.push_back(cv::Point(move_x, move_y));
          break;
        }
      }
      if (isEnd)
      {
        idxdMove = dIDX(move_x, move_y);
        (data + idxdMove)->last_is(NULL);
        break;
      }
    }
    if (oneContour.size() + oneContourOpp.size() > _T_edge_num)
    {
      if (oneContourOpp.size() > 0)
      {
        cv::Point temp;
        for (int i = 0; i < (oneContourOpp.size() + 1) / 2; i++)
        {
          temp = oneContourOpp[i];
          oneContourOpp[i] = oneContourOpp[oneContourOpp.size() - 1 - i];
          oneContourOpp[oneContourOpp.size() - 1 - i] = temp;
        }
        oneContourOpp.insert(oneContourOpp.end(), oneContour.begin(), oneContour.end());
        edgeSegments.push_back(oneContourOpp);
        // not using
        // edgeSegmentsForCluster.push_back(EdgeSegmentYXParam(oneContourOpp));
      }
      else
      {
        edgeSegments.push_back(oneContour);
        // not using
        // edgeSegmentsForCluster.push_back(EdgeSegmentYXParam(oneContour));
      }
    }
  }


  void AAMED::draw_edge_contours(cv::Mat &imgEdge, std::vector<std::vector<cv::Point>> &edgeSegments, std::vector<EdgeSegmentYXParam> &edgeSegmentsForCluster)
  {
    const int clockWise[8][2] = { { 0,1 },{ 1,0 },{ 0,-1 },{ -1,0 },{ -1,1 },{ 1,1 },{ 1,-1 },{ -1,-1 } };
    const int anticlockWise[8][2] = { { 0,-1 },{ 1,0 },{ 0,1 },{ -1,0 },{ -1,-1 },{ 1,-1 },{ 1,1 },{ -1,1 } };
    int idx_first = (iROWS - 1)*iCOLS;

    uchar *_edge = imgEdge.data;
    for (int i = 0; i < iCOLS; i++)
    {
      _edge[i] = 0;
      _edge[idx_first + i] = 0;
    }
    for (int i = 1; i < iROWS - 1; i++)
    {
      _edge[i*iCOLS] = 0;
      _edge[i*iCOLS + iCOLS - 1] = 0;
    }
    for (int i = 1; i < iROWS; i++)
    {
      idx_first = i*iCOLS;
      for (int j = 1; j < iCOLS; j++)
      {
        if (_edge[idx_first + j])
        {
          _edge[idx_first + j] = 0;
          if (_edge[idx_first + iCOLS + j - 1] && _edge[idx_first + iCOLS + j] && _edge[idx_first + iCOLS + j + 1])
            continue;
          else
          {
            trace_pixel(clockWise, anticlockWise, _edge, i, j, edgeSegments, edgeSegmentsForCluster);
          }
        }
      }
    }
  }

  void AAMED::bold_edge(uchar *_edge, const std::vector<std::vector<cv::Point> > &edge_Contours)
  {
    const uchar matBold[9] = { 1,2,3,11,12,13,21,22,23 };
    //const uchar matBold[9] = { 12,12,12,12,12,12,12,12,12 };
    int numContours = int(edge_Contours.size());
    int x, y, idx, idx_i;
    for (int i = 0; i < numContours; i++)
    {
      idx_i = int(edge_Contours[i].size());
      for (int j = 0; j < idx_i; j++)
      {
        // std::cout << "IN: "<< i << " " << j << std::endl;
        idx = edge_Contours[i][j].x*iCOLS + edge_Contours[i][j].y;
        for (int k = 0; k < 9; k++)
        {
          x = matBold[k] / 10; y = matBold[k] - 10 * x;
          x -= 1, y -= 2;
          _edge[idx + x*iCOLS + y] = matBold[k];
        }
      }
    }
  }

  void AAMED::segments_filter(const std::vector<std::vector<cv::Point>> &edgeSegmentsIn,
                              const std::vector<EdgeSegmentYXParam> &edgeSegmentParamsIn,
                              std::vector<std::vector<int>> &clusterRes) {
    // clear the output vector to make sure the result is newest
    clusterRes.clear();

    if (edgeSegmentsIn.size() != edgeSegmentParamsIn.size()) {
      return;
    }
    
    // create the DSU to cluster the segments clusterRes
    int segmentsNum = edgeSegmentParamsIn.size();
    DSU dsu(segmentsNum);
    for (int i = 0; i < segmentsNum; ++i) {
      if (edgeSegmentParamsIn[i].isClosed || edgeSegmentParamsIn[i].length < 20) {
        continue;
      }
      for (int j = i + 1; j < segmentsNum; ++j) {
        int curStatus;
        bool res = segments_are_same_group(edgeSegmentsIn[i], edgeSegmentsIn[j],
                                           edgeSegmentParamsIn[i], edgeSegmentParamsIn[j], curStatus);
        // std::cout << "the segment[" << i << "] and segment [" << j << "] status is: " << curStatus << std::endl;
        if (res) {
          dsu.unite(i, j);
        }
      }
    }
    
    clusterRes.resize(segmentsNum);
    for (int i = 0; i < segmentsNum; ++i) {
        int root = dsu.find(i);
        clusterRes[root].push_back(i);
    }
  }

  bool AAMED::segments_are_same_group(const std::vector<cv::Point>& seg1, const std::vector<cv::Point>& seg2,
                                      const EdgeSegmentYXParam& segParam1, const EdgeSegmentYXParam& segParam2, int& status) {
    //case 1: we dont connect the real short segment
    if (segParam2.length < 20) {
      status = -1;
      return false;
    }

    // case 2: if any of the input segments is closed, they don't need to cluster
    if (segParam1.isClosed || segParam2.isClosed) {
      status = -2;
      return false;
    }

    // case 3: if the included angle between the two segments was too small, pass
    double includedAngle = atan((segParam1.m - segParam2.m) / (1 + segParam1.m * segParam2.m)) * 180.0 / M_PI;
    if (std::fabs(includedAngle) < 22.5) {
      status = -3;
      return false;
    }

    int closeEnoughDIst = 10;
    double d1 = distance_of_2point(segParam1.endPoint, segParam2.startPoint);
    double d2 = distance_of_2point(segParam1.endPoint, segParam2.endPoint);
    double d3 = distance_of_2point(segParam1.startPoint, segParam2.startPoint);
    double d4 = distance_of_2point(segParam1.startPoint, segParam2.endPoint);

    int closeEnoughCount = 0;
    if (d1 < closeEnoughDIst)
      ++closeEnoughCount;
    if (d2 < closeEnoughDIst)
      ++closeEnoughCount;
    if (d3 < closeEnoughDIst)
      ++closeEnoughCount;
    if (d4 < closeEnoughDIst)
      ++closeEnoughCount;

    // case 4: we only accept that the two segments only have one connected point
    if (closeEnoughCount != 1) {
      status = -4;
      return false;
    }

    int overlapCount = 0;
    status = 1;
    // case6: remove two segments that overlap area more than 30%
    if (seg1.size() <= seg2.size()) {
      if (d1 < closeEnoughDIst) {
        for (int i = seg1.size()-1, j = 0; i > 0; --i, ++j) {
          if (distance_of_2point(seg1[i], seg2[j]) < closeEnoughDIst)
            ++overlapCount;
          if (overlapCount > seg1.size() * 0.3) {
            status = -5;
          }
        }
      } else if (d2 < closeEnoughDIst) {
        for (int i = seg1.size()-1, j = seg2.size()-1; i > 0; --i, --j) {
          if (distance_of_2point(seg1[i], seg2[j]) < closeEnoughDIst)
            ++overlapCount;
          if (overlapCount > seg1.size() * 0.3) {
            status = -6;
          }
        }
      } else if (d3 < closeEnoughDIst) {
        for (int i = 0, j = 0; i < seg1.size(); ++i, ++j) {
          if (distance_of_2point(seg1[i], seg2[j]) < closeEnoughDIst)
            ++overlapCount;
          if (overlapCount > seg1.size() * 0.3) {
            status = -7;
          }
        }
      } else if (d4 < closeEnoughDIst) {
        for (int i = 0, j = seg2.size()-1; i < seg1.size(); ++i, --j) {
          if (distance_of_2point(seg1[i], seg2[j]) < closeEnoughDIst)
            ++overlapCount;
          if (overlapCount > seg1.size() * 0.3) {
            status = -8;
          }
        }
      }
    } else {
      if (d1 < closeEnoughDIst) {
        for (int i = 0, j = seg1.size()-1; i < seg2.size(); ++i, --j) {
          if (distance_of_2point(seg2[i], seg1[j]) < closeEnoughDIst)
            ++overlapCount;
          if (overlapCount > seg2.size() * 0.3) {
            status = -9;
          }
        }
      } else if (d2 < closeEnoughDIst) {
        for (int i = seg2.size()-1, j = seg1.size()-1; i > 0; --i, --j) {
          if (distance_of_2point(seg2[i], seg1[j]) < closeEnoughDIst)
            ++overlapCount;
          if (overlapCount > seg2.size() * 0.3) {
            status = -10;
          }
        }
      } else if (d3 < closeEnoughDIst) {
        for (int i = 0, j = 0; i < seg2.size(); ++i, ++j) {
          if (distance_of_2point(seg2[i], seg1[j]) < closeEnoughDIst)
            ++overlapCount;
          if (overlapCount > seg2.size() * 0.3) {
            status = -11;
          }
        }
      } else if (d4 < closeEnoughDIst) {
        for (int i = seg2.size()-1, j = 0; i > 0; --i, ++j) {
          if (distance_of_2point(seg2[i], seg1[j]) < closeEnoughDIst)
            ++overlapCount;
          if (overlapCount > seg2.size() * 0.3) {
            status = -12;
          }
        }
      }
    }
    if (status == 1) {
      return true;
    } else {
      return false;
    }
  }
#include "aamed/AAMED.h"

void AAMED::clockwise_contour(std::vector<cv::Point> &antiContour)
{
  cv::Point dot_ed;
  dot_ed = antiContour[antiContour.size() - 1];
  int idxdHead = dIDX(antiContour[0].x, antiContour[0].y);
  int idxdTail = dIDX(dot_ed.x, dot_ed.y);
  if (data[idxdHead].lastAddress != NULL)//如果非空，则选择起点为下一点
  {
    antiContour[0] = data[idxdHead].nextAddress->Location;//选择起点为下一点
    data[idxdHead].nextAddress->lastAddress = NULL;
    data[idxdHead].nextAddress = NULL;
    idxdHead = dIDX(antiContour[0].x, antiContour[0].y);
  }
  if (data[idxdTail].nextAddress != NULL)//如果不为空，则选择起点为前一点
  {
    dot_ed = data[idxdTail].lastAddress->Location;
    antiContour[antiContour.size() - 1] = dot_ed;
    data[idxdTail].lastAddress->nextAddress = NULL;
    data[idxdTail].lastAddress = NULL;
    idxdTail = dIDX(dot_ed.x, dot_ed.y);
  }
  //至此，起点和终点均已预处理完毕，只需要对整个弧段进行交换处理即可
  Node_FC * SigPot_Temp = NULL;
  cv::Point* Idx_Dot = NULL;
  for (int step_idx = idxdTail; step_idx != idxdHead;)
  {
    SigPot_Temp = data[step_idx].nextAddress;
    data[step_idx].nextAddress = data[step_idx].lastAddress;
    data[step_idx].lastAddress = SigPot_Temp;
    Idx_Dot = &(data[step_idx].nextAddress->Location); 
    // std::cout << "clockwise cont" << std::endl;
    step_idx = Idx_Dot->x*dCOLS + Idx_Dot->y; 
  }//最后一个点没有交换		
  data[idxdHead].lastAddress = data[idxdHead].nextAddress;
  data[idxdHead].nextAddress = NULL;
  //后面参与计算均与起点终点有关，基本不会再涉及到内部矩阵元素，因此交换首尾重要信息
  int ID_temp = data[idxdTail].edgeID;
  double fit_sig_temp;
  data[idxdTail].edgeID = data[idxdHead].edgeID;
  data[idxdHead].edgeID = ID_temp;
  for (uint mm = 0; mm < MAT_NUMBER; mm++)
  {
    fit_sig_temp = data[idxdTail].nodesMat[mm];
    data[idxdTail].nodesMat[mm] = data[idxdHead].nodesMat[mm];
    data[idxdHead].nodesMat[mm] = fit_sig_temp;
  }
  for (uint mm = 0; mm < antiContour.size() / 2; mm++)
  {
    dot_ed = antiContour[mm];
    antiContour[mm] = antiContour[antiContour.size() - 1 - mm];
    antiContour[antiContour.size() - 1 - mm] = dot_ed;
  }
} // end of clockwise_contour


void AAMED::check_FSA(std::vector<cv::Point> &dpContour, 
    std::vector<std::vector<cv::Point>> &FSA_ArcContours)
{
  const double Theta_fsa = _theta_fsa, Length_fsa = _length_fsa, Length_fsa_inv = 1 / _length_fsa;

  
  cv::Vec4i temp_line;
  cv::Point Ai[3], ni[2];
  cv::Point2f FSA;
  float tp[3], l_Ai_1Ai;
  int dir;
  const int dpContourNum = dpContour.size();
  const double Length_fsa2 = Length_fsa*Length_fsa, Length_fsa_inv2 = Length_fsa_inv*Length_fsa_inv;



  for (int i = 1; i < dpContourNum; i++)
  {
    // std::cout << " -- Pre -- " << std::endl; 
    // oneContour.clear();
    std::vector<cv::Point> oneContour;
    Ai[0] = dpContour[i - 1]; Ai[1] = dpContour[i];
    ni[0] = Ai[1] - Ai[0];
    l_Ai_1Ai = ni[0].x*ni[0].x + ni[0].y*ni[0].y;
    if (l_Ai_1Ai < 5)
      continue;
    if (i + 1 >= dpContour.size())
    {
      continue;
    }
    Ai[2] = dpContour[i + 1];
    ni[1] = Ai[2] - Ai[1];
    dir = ni[0].x*ni[1].y - ni[0].y*ni[1].x;
    FSA.x = (ni[0].x*ni[1].x + ni[0].y*ni[1].y) / l_Ai_1Ai; FSA.y = abs(dir) / l_Ai_1Ai;
    tp[0] = FSA.x - tan(CV_PI / 2 - Theta_fsa)*FSA.y;
    tp[1] = FSA.y / sin(Theta_fsa);
    tp[2] = FSA.x*FSA.x + FSA.y*FSA.y;
    if (tp[0] <= 0 || tp[1] <= 0 || tp[2]<Length_fsa_inv2 || tp[2]>Length_fsa2)
    {
      continue;
    }
    dir = dir > 0 ? 1 : -1;
    i++;

    oneContour.push_back(Ai[0]), oneContour.push_back(Ai[1]), oneContour.push_back(Ai[2]);
    for (int j = i + 1; j < dpContour.size(); j++, i = j - 1)
    {
      ni[0] = ni[1];
      l_Ai_1Ai = ni[0].x*ni[0].x + ni[0].y*ni[0].y;
      ni[1] = dpContour[j] - dpContour[j - 1];
      FSA.x = (ni[0].x*ni[1].x + ni[0].y*ni[1].y) / l_Ai_1Ai;
      FSA.y = dir*(ni[0].x*ni[1].y - ni[0].y*ni[1].x) / l_Ai_1Ai;
      tp[0] = FSA.x - tan(CV_PI / 2 - Theta_fsa)*FSA.y;
      tp[1] = FSA.y / sin(Theta_fsa);
      tp[2] = FSA.x*FSA.x + FSA.y*FSA.y;
      if (tp[0] > 0 && tp[1] > 0 && tp[2] > Length_fsa_inv2 &&tp[2] < Length_fsa2)
      {
        oneContour.push_back(dpContour[j]);
        continue;
      }
      else
        break;
    }
    // std::cout << " -- Pos -- " << dpContour.size() << " i: " << i << std::endl; 
    if (oneContour.size() < PARAMS->MIN_DP_CONTOUR_NUM()){
      // std::cout << "skipped at MIN_DP_CONTOUR_NUM: " << MIN_DP_CONTOUR_NUM << std::endl;
      continue;
    }

    if (dir > 0)
      // std::cout << "need to clock wise " << i << std::endl;  // Comment This line for best performance
      clockwise_contour(oneContour);

    cv::Point st, ed;
    st = oneContour[0], ed = oneContour.back();
    int arc_edge_num = data[dIDX(ed.x, ed.y)].edgeID - data[dIDX(st.x, st.y)].edgeID;
    // if (arc_edge_num > 0)
      // std::cout << "arc_edge_num" << arc_edge_num << std::endl;
    
  // std::cout << "------------- " << i << std::endl;
  // std::cout << "arc_edge_num " << arc_edge_num << std::endl;
  // std::cout << "thresh " << _T_edge_num/2*oneContour.size() << std::endl;


    if (arc_edge_num < _T_edge_num/2*oneContour.size()){
      // std::cout << "skipped at last" << std::endl;
      continue;
    }
    FSA_ArcContours.push_back(oneContour);
    // std::cout << "pushing" << std::endl;
  } // End of for
} // End of check_FSA

void AAMED::fsa_segments(std::vector<std::vector<cv::Point>> &dpSegments, 
                            std::vector<std::vector<cv::Point>> &fsaSegments)
{
  for (int i = 0; i < dpSegments.size(); i++)
  {
    check_FSA(dpSegments[i], fsaSegments);
  }
}
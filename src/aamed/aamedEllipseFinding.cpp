// #include "tagDetector.h"
#include "aamed/AAMED.h"

// namespace CopperTag{

  void AAMED::sort_arcs(std::vector<std::vector<cv::Point>> &detArcs)
  {
    int arc_num = detArcs.size();
    std::vector<int> arcs_edge_num(arc_num);
    cv::Point st, ed;
    for (int i = 0; i < arc_num; i++)
    {
      st = detArcs[i][0];
      ed = detArcs[i].back();
      arcs_edge_num[i] = data[dIDX(ed.x, ed.y)].edgeID - data[dIDX(st.x, st.y)].edgeID;
    }


    Node_FC *_tmpdata = data;
    int tmpdCols = dCOLS;
    sort(detArcs.begin(), detArcs.end(),
      [&_tmpdata, &tmpdCols](std::vector<cv::Point> &a1, std::vector<cv::Point> &a2) 
    {
      cv::Point st, ed;
      st = a1[0], ed = a1.back();
      int arc1_edge_num = _tmpdata[ed.x*tmpdCols+ed.y].edgeID - _tmpdata[st.x*tmpdCols + st.y].edgeID;

      st = a2[0], ed = a2.back();
      int arc2_edge_num = _tmpdata[ed.x*tmpdCols + ed.y].edgeID - _tmpdata[st.x*tmpdCols + st.y].edgeID;

      return arc1_edge_num > arc2_edge_num;
    }
    );
  }


  void AAMED::create_arc_search_region(std::vector<std::vector<cv::Point>> &fsaarcs)
  {
    const int arc_num = fsaarcs.size();
    cv::Point *_arc_data = NULL;
    int subarc_num;
    ArcSearchRegion asrTemp;

    //	asrs.resize(arc_num);
    for (int i = 0; i < arc_num; i++)
    {
      subarc_num = fsaarcs[i].size();
      _arc_data = fsaarcs[i].data();
      asrTemp.create(_arc_data, _arc_data + 1, _arc_data + subarc_num - 2, _arc_data + subarc_num - 1);
      asrs.push_back(asrTemp);
    }
  }


  void AAMED::get_arc_kdTree(std::vector<std::vector<cv::Point> > &arcs)
  {
    source_arcs.create(arcs.size(), 2, CV_32FC1);
    cv::Point *temp;
    float *_source_arcs = (float*)source_arcs.data;
    int source_num = arcs.size();
    for (int i = 0; i < source_num; i++)
    {
      temp = &arcs[i][0];
      _source_arcs[i << 1] = temp->x;
      _source_arcs[(i << 1) + 1] = temp->y;
    }
    kdTree_Arcs.build(source_arcs, indexParams, cvflann::FLANN_DIST_L1);
  }


  void AAMED::arcs_grouping(std::vector<std::vector<cv::Point>> &fsaarcs)
  {
    int fsaarcs_num = fsaarcs.size();
    int sub_arcnum, dist_l, searchNum, findIdx, linkval[2], group_res, _linkIdx;
    //const int maxArcs = std::min(32, fsaarcs_num);
    const int maxArcs = fsaarcs_num; // std::min(32, fsaarcs_num);
    int *_indices_arcs = NULL;
    float *_dist_arcs = NULL;
    cv::Point *l1[4] = { NULL,NULL,NULL,NULL }, *l2[4] = { NULL,NULL,NULL,NULL };


    cv::Point l1_l2_O, vecl[2], l1_l2[3], arc_n[3];

    unsigned char distType = 0x00;
    cv::Point vecFeature[8];

    LinkMatrix.Update(fsaarcs_num*fsaarcs_num);// 更新邻接矩阵个数
    char* _LinkMatrix = LinkMatrix.GetDataPoint();

    int T_ij, T_ji;
    cv::Point *_vec_data_i = NULL, *_vec_data_j = NULL; //获取vector的数据指针


    // 1-25 Add
    ArcSearchRegion asr1, asr2;
    ArcSearchRegion *_asrs = asrs.data(), *_asr1(NULL), *_asr2(NULL);

    for (int i = 0; i < fsaarcs_num; i++)
    {
      sub_arcnum = int(fsaarcs[i].size());
      _vec_data_i = fsaarcs[i].data();

      l1[0] = _vec_data_i;                   // A^{i}_1
      l1[1] = _vec_data_i + 1;               // A^{i}_2
      l1[2] = _vec_data_i + sub_arcnum - 2;  // A^{i}_{-2}
      l1[3] = _vec_data_i + sub_arcnum - 1;  // A^{i}_{-1}

      //asr1.create(l1[0], l1[1], l1[2], l1[3]); // Create Search Region, Modification of Prasad
      _asr1 = _asrs + i;
      vecFeature[0].x = l1[1]->x - l1[0]->x; vecFeature[0].y = l1[1]->y - l1[0]->y; //A0→A1
      vecFeature[1].x = l1[3]->x - l1[2]->x; vecFeature[1].y = l1[3]->y - l1[2]->y; //Aend-1Aend
      vecFeature[2].x = l1[3]->x - l1[0]->x; vecFeature[2].y = l1[3]->y - l1[0]->y;

      dist_l = abs(l1[0]->x - l1[3]->x) + abs(l1[0]->y - l1[3]->y); //KD-Tree Search Radius.
      query_arcs[0] = l1[3]->x; query_arcs[1] = l1[3]->y; // KD-Tree Search root.
      //利用kd树，搜索满足距离约束的点
      searchNum = kdTree_Arcs.radiusSearch(query_arcs, indices_arcs, dist_arcs, dist_l, maxArcs);
      //if (i == 0)
      //{
      //	cout << "Begin Error" << endl;
      //	cout << query_arcs[0] << "," << query_arcs[1] << endl;
      //	cout << indices_arcs << endl;
      //	cout << dist_arcs << endl;
      //	cout << dist_l << endl;
      //	cout << maxArcs << endl;

      //	float *_source_arcs = (float*)source_arcs.data;
      //	if (source_arcs.rows > 13)
      //	{
      //		cout << _source_arcs[13 * 2] << " , " << _source_arcs[13 * 2 + 1] << endl;
      //		//system("pause");
      //	}
      //	//kdTree_Arcst.radiusSearch(query_arcs, indices_arcst, dist_arcst, dist_l, maxArcs);
      //	//cout << "New KDTree" << endl;
      //	//cout << indices_arcst << endl;
      //	//cout << dist_arcst << endl;
      //	
      //}

      _indices_arcs = (int*)indices_arcs.data;
      _dist_arcs = (float*)dist_arcs.data;

      for (int j = 0; j < searchNum; j++)
      {
        distType = FLED_GROUPING_IBmA1_IAnB1;
        findIdx = _indices_arcs[j];// 找到的弧段编号
        _linkIdx = i*fsaarcs_num + findIdx; //获取邻接矩阵的编号
        if (_LinkMatrix[_linkIdx] != 0)
          continue;
        int findArcNum = fsaarcs[findIdx].size();
        _vec_data_j = fsaarcs[findIdx].data();   //获取被搜索的弧段指针

        l2[0] = _vec_data_j;                   // A^{j}_1
        l2[1] = _vec_data_j + 1;               // A^{j}_2
        l2[2] = _vec_data_j + findArcNum - 2;  // A^{j}_{-2}
        l2[3] = _vec_data_j + findArcNum - 1;  // A^{j}_{-1}

        //asr2.create(l2[0], l2[1], l2[2], l2[3]); // Create Search Region, Modification of Prasad
        _asr2 = _asrs + findIdx;
        vecFeature[3].x = l2[0]->x - l1[3]->x; vecFeature[3].y = l2[0]->y - l1[3]->y;
        vecFeature[7].x = l1[0]->x - l2[3]->x; vecFeature[7].y = l1[0]->y - l2[3]->y;

        T_ij = std::min(abs(vecFeature[1].x) + abs(vecFeature[1].y), // |A^{i}_{-1} - A^{i}_{-2}|
          abs(l2[0]->x - l2[1]->x) + abs(l2[0]->y - l2[1]->y));    // |A^{k}_1 - A^{k}_2|
        T_ji = std::min(abs(vecFeature[0].x) + abs(vecFeature[0].x), // |A^{i}_1 - A^{i}_2|
          abs(l2[2]->x - l2[3]->x) + abs(l2[2]->y - l2[3]->y));    // |A^{k}_{-1} - A^{k}_{-2}|
        if (_dist_arcs[j] < T_ij)
          distType = distType | FLED_GROUPING_CAnB1;
        if (abs(vecFeature[7].x) + abs(vecFeature[7].y) < T_ji)
          distType = distType | FLED_GROUPING_CBmA1;

        switch (distType)
        {
        case FLED_GROUPING_FBmA1_FAnB1:// 搜索点与尾点宽度都较远
        {
          //group_res = CASE_1(&asr1, &asr2);
          group_res = CASE_1(_asr1, _asr2);
          //case_stat[0]++;
          //group_res = Group4FAnB1_FBmA1(vecFeature, l1, l2);
          //group_res = Group4FAnB1_FBmA1(vecFeature, l1, l2, &asr1, &asr2);
          break;
        }
        case FLED_GROUPING_FBmA1_CAnB1:// 搜索点较近，尾点较远
        {
          group_res = Group4CAnB1_FBmA1(vecFeature, l1, l2);
          //case_stat[1]++;
          break;
        }
        case FLED_GROUPING_CBmA1_FAnB1:// 搜索点较远，尾点较近
        {
          group_res = Group4FAnB1_CBmA1(vecFeature, l1, l2);
          //case_stat[2]++;
          break;
        }
        case FLED_GROUPING_CBmA1_CAnB1: // 搜索点与尾点宽度都较近
        {
          if (findIdx == i)
          {
            group_res = 1;
            for (int j = 0; j < fsaarcs_num; j++)
            {
              _LinkMatrix[i*fsaarcs_num + j] = _LinkMatrix[j*fsaarcs_num + i] = -1;
            }
          }
          else
            group_res = Group4CAnB1_CBmA1(vecFeature, l1, l2);

          //case_stat[3]++;

          break;
        }
        default:
          break;
        }
        //if (i == 25 && findIdx == 32)
        //	cout << group_res << endl;
        //if (i == 32 && findIdx == 25)
        //	cout << group_res << endl;
        if (findIdx == i&&group_res == -1) //若弧段i不能跟自己相连，则这个弧段不可能跟其余的相连
        {
          continue;
        }


        _LinkMatrix[_linkIdx] = group_res;
        if (group_res == -1)
          _LinkMatrix[findIdx*fsaarcs_num + i] = -1;
        //if (findIdx == i&&group_res==-1) //若弧段i不能跟自己相连，则这个弧段不可能跟其余的相连
        //{
        //	for (int j = 0; j < fsaarcs_num; j++)
        //	{
        //		_LinkMatrix[i*fsaarcs_num + j] = _LinkMatrix[j*fsaarcs_num + i] = -1;
        //	}
        //}


      }
    }
  }


  void AAMED::get_linked_arcs(const char *_linkMatrix, int arc_num)
  {
    lA.resize(arc_num);
    //查找第i个弧段相连的弧段和不相连的
    for (int i = 0; i < arc_num; i++)
    {
      //if (i == 25)
      //	cout << "Begin Error" << endl;
      int idx = i*arc_num;
      lA[i].clear();
      for (int j = 0; j < arc_num; j++)
      {
        if (_linkMatrix[idx + j] == 1) // i 与j相连，则连接
          lA[i].idx_linking.push_back(j);
        else if (_linkMatrix[idx + j] == -1) //i与j不连，则不连
          lA[i].idx_notlink.push_back(j);
        if (_linkMatrix[j*arc_num + i] == 1)
          lA[i].idx_linked.push_back(j);
      }
    }
  }


  void AAMED::get_arc_matrix(std::vector<cv::Vec<double, MAT_NUMBER>> &arcfitmat, 
    std::vector<std::vector<cv::Point>> &fsa_arc)
  {
    const int arcs_num = fsa_arc.size();
    cv::Vec<double, MAT_NUMBER> *_arcs_data = NULL, *arc_temp = NULL;
    cv::Point *st, *ed;
    int idx_st, idx_ed;
    Node_FC *st_node = NULL, *ed_node = NULL;

    arcfitmat.resize(arcs_num);
    _arcs_data = arcfitmat.data();

    for (int i = 0; i < arcs_num; i++)
    {
      arc_temp = _arcs_data + i;
      st = &fsa_arc[i][0]; ed = &fsa_arc[i].back();
      idx_st = dIDX(st->x, st->y); idx_ed = dIDX(ed->x, ed->y);
      st_node = data + idx_st; ed_node = data + idx_ed;
      for (int j = 0; j < MAT_NUMBER; j++)
        arc_temp->val[j] = ed_node->nodesMat[j] - st_node->nodesMat[j];
    }
  }

// }
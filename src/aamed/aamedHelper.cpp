// #include "tagDetector.h"
#include "aamed/AAMED.h"

// namespace CopperTag{
  using namespace std;
  using namespace cv;
	//the region constraint and curvature constraint are used to calculate Lij in four cases
    char AAMED::Group4FAnB1_FBmA1(Point vecFet[8], const Point * const *l1, const Point * const *l2, const ArcSearchRegion * const asri, const ArcSearchRegion * const asrk) const
    {
      int linkval[2];



      if (!RegionConstraint(asri, asrk))
        return -1;
      if (!RegionConstraint(asrk, asri))
        return -1;
      return 1;

      //if (!CONSTRAINT_GLOBAL_REGION(vecFet, l1, l2[0]))
      //	return -1;
      //if (!CONSTRAINT_GLOBAL_REGION(vecFet, l1, l2[3]))
      //	return -1;
      vecFet[5].x = l2[3]->x - l2[2]->x; vecFet[5].y = l2[3]->y - l2[2]->y;
      vecFet[4].x = l2[1]->x - l2[0]->x; vecFet[4].y = l2[1]->y - l2[0]->y;

      linkval[0] = CONSTRAINT_GLOABL_CURVATURE(vecFet, 1);
      if (linkval[0] == -1)
        return -1;
      linkval[1] = CONSTRAINT_GLOABL_CURVATURE(vecFet, 2);
      if (linkval[1] == -1)
        return -1;
      if (linkval[0] == 1 || linkval[1] == 1)
        return 1;
      return 0;
    }
    char AAMED::Group4FAnB1_FBmA1(Point vecFet[8], const Point * const *l1, const Point * const *l2) const 
    {
      int linkval[2];
      if (!CONSTRAINT_GLOBAL_REGION(vecFet, l1, l2[0]))
        return -1;
      if (!CONSTRAINT_GLOBAL_REGION(vecFet, l1, l2[3]))
        return -1;
      vecFet[5].x = l2[3]->x - l2[2]->x; vecFet[5].y = l2[3]->y - l2[2]->y;
      vecFet[4].x = l2[1]->x - l2[0]->x; vecFet[4].y = l2[1]->y - l2[0]->y;

      linkval[0] = CONSTRAINT_GLOABL_CURVATURE(vecFet, 1);
      if (linkval[0] == -1)
        return -1;
      linkval[1] = CONSTRAINT_GLOABL_CURVATURE(vecFet, 2);
      if (linkval[1] == -1)
        return -1;
      if (linkval[0] == 1 || linkval[1] == 1)
        return 1;
      return 0;
    }
    char AAMED::Group4CAnB1_FBmA1(Point vecFet[8], const Point * const *l1, const Point * const *l2) const
    {
      Point l1_l2_O, vecl[2];
      l1_l2_O.x = (vecFet[3].x >> 1) + l1[3]->x; l1_l2_O.y = (vecFet[3].y >> 1) + l1[3]->y;
      vecl[0].x = l1_l2_O.x - l1[2]->x; vecl[0].y = l1_l2_O.y - l1[2]->y;
      vecl[1].x = l2[1]->x - l1_l2_O.x; vecl[1].y = l2[1]->y - l1_l2_O.y;
      if (!CONSTRAINT_NEIBOR_FSA(vecl))
        return -1;
      vecFet[5].x = l2[3]->x - l2[2]->x; vecFet[5].y = l2[3]->y - l2[2]->y;
      if (!CONSTRAINT_NEIBOR_TAIL(vecFet))
        return -1;
      
      return 1;
    }
    char AAMED::Group4FAnB1_CBmA1(Point vecFet[8], const Point * const *l1, const Point * const *l2) const
    {
      if (!CONSTRAINT_GLOBAL_REGION(vecFet, l1, l2[0]))
        return -1;
      vecFet[5].x = l2[3]->x - l2[2]->x; vecFet[5].y = l2[3]->y - l2[2]->y;
      vecFet[4].x = l2[1]->x - l2[0]->x; vecFet[4].y = l2[1]->y - l2[0]->y;
      if (CONSTRAINT_GLOABL_CURVATURE(vecFet, 1) == -1)
        return -1;
      return 1;
    }
    char AAMED::Group4CAnB1_CBmA1(Point vecFet[8], const Point * const *l1, const Point * const *l2) const
    {
      Point l1_l2_O, vecl[2];
      l1_l2_O.x = (vecFet[3].x >> 1) + l1[3]->x; l1_l2_O.y = (vecFet[3].y >> 1) + l1[3]->y;
      vecl[0].x = l1_l2_O.x - l1[2]->x; vecl[0].y = l1_l2_O.y - l1[2]->y;
      vecl[1].x = l2[1]->x - l1_l2_O.x; vecl[1].y = l2[1]->y - l1_l2_O.y;
      if (!CONSTRAINT_NEIBOR_FSA(vecl))// These two arcs do not satisfy the curvature contraint, i.e. Lij = -1;
        return -1;
      else
        return 1;
    }

    //Neighbor Grouping: FSA Constraint
    bool AAMED::CONSTRAINT_NEIBOR_FSA(const Point ni[2]) const
    {
      const double Theta_fsa = _theta_fsa, Length_fsa = _length_fsa, Length_fsa_inv = 1 / _length_fsa;

      cv::Point2d FSA;
      double l_Ai_1Ai = ni[0].x*ni[0].x + ni[0].y*ni[0].y, tp[3];
      FSA.x = (ni[0].x*ni[1].x + ni[0].y*ni[1].y) / l_Ai_1Ai;
      FSA.y = (ni[1].x*ni[0].y - ni[1].y*ni[0].x) / l_Ai_1Ai;
      tp[0] = FSA.x - tan(CV_PI / 2 - Theta_fsa)*FSA.y;
      tp[1] = FSA.y / sin(Theta_fsa);
      tp[2] = FSA.x*FSA.x + FSA.y*FSA.y;
      if (tp[0] > 0 && tp[1] > 0 && tp[2] > Length_fsa_inv*Length_fsa_inv&&tp[2] < Length_fsa*Length_fsa)
        return true;
      else
        return false;
    }
    bool AAMED::CONSTRAINT_NEIBOR_TAIL(const Point AB[8]) const
    {
      int res = AB[5].x*AB[7].y - AB[5].y*AB[7].x;
      if (res > 0)
        return false;
      res = AB[7].x*AB[0].y - AB[7].y*AB[0].x;
      if (res > 0)
        return false;
      return true;
    }
    bool AAMED::CONSTRAINT_GLOBAL_REGION(const Point vecFet[8], const Point * const *l1, const Point *checkPoint) const 
    {
      Point checkVec;
      int checkVal;
      checkVec.x = checkPoint->x - l1[0]->x; checkVec.y = checkPoint->y - l1[0]->y;
      checkVal = -checkVec.x*vecFet[0].y + checkVec.y*vecFet[0].x;
      if (checkVal > 0)
        return false;
      checkVal = checkVec.x*vecFet[2].y - checkVec.y*vecFet[2].x;
      if (checkVal < 0)
        return false;
      checkVec.x = checkPoint->x - l1[3]->x; checkVec.y = checkPoint->y - l1[3]->y;
      checkVal = checkVec.x*vecFet[1].y - checkVec.y*vecFet[1].x;
      if (checkVal < 0)
        return false;
      return true;
    }
    int AAMED::CONSTRAINT_GLOABL_CURVATURE(const Point vecFet[8], const int flag) const
    {
      int val[3];
      if (flag == 1)
      {
        val[1] = vecFet[1].x*vecFet[3].y - vecFet[1].y*vecFet[3].x;
        if (val[1] > 0)
          return -1;
        val[2] = vecFet[3].x*vecFet[4].y - vecFet[3].y*vecFet[4].x;
        if (val[2] > 0)
          return -1;
        val[0] = vecFet[1].x*vecFet[4].y - vecFet[1].y*vecFet[4].x;
        if (abs(val[0]) < 1e-8)
        {
          val[0] = vecFet[1].x*vecFet[4].x + vecFet[1].y*vecFet[4].y;
          if (val[0] > 0)
            return -1;
          else
            return 0;
        }
        else
        {
          if (val[0] < 0)
            return 1;
          else
          {
            if (val[0] + val[1] < 0 && val[0] + val[2] < 0)
              return 0;
            else
              return -1;
          }
        }
        return 0;
      }
      else
      {
        val[1] = vecFet[5].x*vecFet[7].y - vecFet[5].y*vecFet[7].x;
        if (val[1] > 0)
          return -1;
        val[2] = vecFet[7].x*vecFet[0].y - vecFet[7].y*vecFet[0].x;
        if (val[2] > 0)
          return -1;
        val[0] = vecFet[5].x*vecFet[0].y - vecFet[5].y*vecFet[0].x;
        if (abs(val[0]) < 1e-8)
        {
          val[0] = vecFet[5].x*vecFet[0].x + vecFet[5].y*vecFet[0].y;
          if (val[0] > 0)
            return -1;
          else
            return 0;
        }
        else
        {
          if (val[0] < 0)
            return 1;
          else
          {
            if (val[0] + val[1] < 0 && val[0] + val[2] < 0)
              return 0;
            else
              return -1;
          }
        }
        return 0;
      }
    }
    int AAMED::CONSTRAINT_GLOABL_CURVATURE(const Point arc_n[3]) const
    {
      int val[3];
      val[1] = arc_n[0].x*arc_n[2].y - arc_n[0].y*arc_n[2].x;
      if (val[1] > 0)
        return -1;
      val[2] = arc_n[2].x*arc_n[1].y - arc_n[2].y*arc_n[1].x;
      if (val[2] > 0)
        return -1;
      val[0] = arc_n[0].x*arc_n[1].y - arc_n[0].y*arc_n[1].x;
      if (abs(val[0]) < 1e-8)
      {
        val[0] = arc_n[0].x*arc_n[1].x + arc_n[0].y*arc_n[1].y;
        if (val[0] > 0)
          return -1;
        else
          return 0;
      }
      else
      {
        if (val[0] < 0)
          return 1;
        else
        {
          if (val[0] + val[1] < 0 && val[0] + val[2] < 0)
            return 0;
          else
            return -1;
        }
      }
      return 0;
    }

    bool AAMED::RegionConstraint(const ArcSearchRegion * const asri, const ArcSearchRegion * const asrk) const
    {
      if (!(asri->is_in_search_region(&asrk->A_1)))
        return false;
      if (!(asrk->is_in_search_region(&asri->A__1)))
        return false;
      return true;
    }
    bool AAMED::FSAConstraint(const ArcSearchRegion * const asri, const ArcSearchRegion * const asrk) const
    {
      const double theta_fsa = _theta_fsa, length_fsa = _length_fsa, length_fsa_inv = 1 / _length_fsa;

      cv::Point2d vi, A0_ik, ni, n_i_1;
      double lni2, t, p, lvi;
      A0_ik.x = (asri->A__1.x + asrk->A_1.x) / 2.0, A0_ik.y = (asri->A__1.y + asrk->A_1.y) / 2.0;
      ni.x = A0_ik.x - asri->A__2.x, ni.y = A0_ik.y - asri->A__2.y;
      n_i_1.x = asrk->A_2.x - A0_ik.x, n_i_1.y = asrk->A_2.y - A0_ik.y;

      lni2 = ni.x*ni.x + ni.y*ni.y;
      vi.x = (ni.x*n_i_1.x + ni.y*n_i_1.y) / lni2;
      vi.y = -(ni.x*n_i_1.y - ni.y*n_i_1.x) / lni2;
      t = vi.x - tan(CV_PI / 2 - theta_fsa)*vi.y;
      p = vi.y / sin(theta_fsa);
      if (t <= 0 || p <= 0)
        return false;
      lvi = vi.x*vi.x + vi.y*vi.y;
      if (lvi <= length_fsa_inv*length_fsa_inv || lvi >= length_fsa*length_fsa)
        return false;
      return true;
    }
    // l_ik >= T_ik && l_ki >= T_ki
    char AAMED::CASE_1(const ArcSearchRegion * const asri, const ArcSearchRegion * const asrk) const
    {
      if (!RegionConstraint(asri, asrk))
        return -1;
      if (!RegionConstraint(asrk, asri))
        return -1;
      return 1;
    }
    // l_ik >= T_ik && l_ki < T_ki
    char AAMED::CASE_2(const ArcSearchRegion * const asri, const ArcSearchRegion * const asrk) const
    {
      if (!RegionConstraint(asri, asrk))
        return -1;
      return 1;
    }

// }
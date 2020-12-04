#include "rrt.h"

// ***************************************************************************
rrt::rrt(int nNodes, double minBnds[3], double maxBnds[3], double radNear, double delDist, double radRob, int failItr, DynamicEDTOctomap* octDist)
{
  octDist_ = octDist;
  init(nNodes, minBnds, maxBnds, radNear, delDist, radRob, failItr);
}

// ***************************************************************************
void rrt::init(int nNodes, double minBnds[3], double maxBnds[3], double radNear, double delDist, double radRob, int failItr)
{
  posNds_.resize(nNodes,3);
  cstNds_.resize(nNodes);
  idPrnts_.reserve(nNodes);

  std::memcpy(minBnds_, minBnds, sizeof(double)*3);
  std::memcpy(maxBnds_, maxBnds, sizeof(double)*3);

  radNear_ = radNear;
  nearNds_.reserve(nNodes);

  delDist_ = delDist;
  nNodes_ = nNodes;

  radRob_ = radRob;

  failItr_ = failItr;
}

// ***************************************************************************
void rrt::clear()
{
  //std::cout << "Clearing" << std::endl;
  actNds_ = 0;
  idPrnts_.resize(0);
  //std::cout << "Cleared" << std::endl;
}

// ***************************************************************************
void rrt::modify_node(int idNd, Eigen::Vector3d pos, double cost, int idPrnt)
{
  posNds_.row(idNd) = pos;
 
  cstNds_(idNd) = cost;

  idPrnts_[idNd] = idPrnt;
}

// ***************************************************************************
int rrt::add_node(Eigen::Vector3d pos, double cost, int idPrnt)
{
  //std::cout << "Adding node" << std::endl;
  posNds_.row(actNds_) = pos;
  //std::cout << "Node pos assigned" << std::endl;
 
  cstNds_(actNds_) = cost;

  idPrnts_[actNds_] = idPrnt;
  
  actNds_++;

  return (actNds_-1);
}

// ***************************************************************************
Eigen::MatrixXd rrt::get_path(int idLeaf)
{
  if(actNds_ == 0)
    return Eigen::MatrixXd(0,3);

  Eigen::MatrixXd path(actNds_,3);

  int id = idLeaf;
  path.row(actNds_-1) = posNds_.row(id);

  if(id == 0)
    return path.bottomRows(1);

  int itr = actNds_-2;
  do
  {
    id = idPrnts_[id];
    path.row(itr) = posNds_.row(id);
    itr = itr - 1;
  }
  while(id != 0);

  return path.bottomRows(actNds_-itr-1);
}

// ***************************************************************************
std::vector<int> rrt::get_leaves(int actNodes)
{
  std::vector<int> idLvs;
  
  for(int id=0; id<actNodes; id++)
  {
    if(std::find (idPrnts_.begin(), idPrnts_.end(), id) != idPrnts_.end())
      idLvs.push_back(id);
  }

  return idLvs;
}

// ***************************************************************************
void rrt::build(Eigen::Vector3d posRoot)
{
  //std::cout << "Building tree" << std::endl;
  clear();
  add_node(posRoot, 0, 0);

  //std::cout << "Added root node at position " << posRoot << std::endl;

  int itr = 0;
  while (actNds_ < nNodes_)
  {
    //print_tree();
    Eigen::Vector3d posRand = rand_pos(minBnds_, maxBnds_);
    //std::cout << "Drawn random node at position " << posRand.transpose() << std::endl;

    int idNearest = find_nearest(posRand);
    Eigen::Vector3d posNearest = posNds_.row(idNearest);
    //std::cout << "Nearest node found at position " << posNearest.transpose() << std::endl;

    Eigen::Vector3d posNew = steer(posNearest, posRand, delDist_);
    //std::cout << "New node at position " << posNew.transpose() << std::endl;

    //getchar();

    itr++;
    if (itr > failItr_)
    {
      std::cout << "No solution found in max iterations!" << std::endl;
      break;
    }
    if ( u_coll(posNearest, posNew) )
      continue;
    itr = 0;

    //std::cout << "Nearest to new line is collision-free" << std::endl;

    find_near(posNew, radNear_); // copy idsNear_,pCosts,lCosts_
    //std::cout << "Found near nodes to the new node" << std::endl;

    //print_near();

    //getchar();

    double cstNew = cstNds_(idNearest) + delDist_;    
    int idNew = add_node(posNew, cstNew, idNearest);    

    int idMin = idNearest;
    for (int i=0; i<nearNds_.size(); i++)
    {
      int idNear = nearNds_[i].id;
      double cstL = nearNds_[i].cstLnk; // line (with new) cost
      double cstN = nearNds_[i].cstNd; // node cost

      Eigen::Vector3d posNear = posNds_.row(idNear);

      double potCstNew = cstN + cstL;
      if( potCstNew < cstNew )
      {
        //cstNew = potCstNew;
        modify_node(idNew, posNew, potCstNew, idNear);

        idMin = idNear;
        break; // check if any solution is fine or the least has to be found
      }
    }

    cstNew = cstNds_(idNew);
    for (int i=0; i<nearNds_.size(); i++)
    {
      int idNear = nearNds_[i].id;
      double cstL = nearNds_[i].cstLnk; // line (with new) cost
      double cstN = nearNds_[i].cstNd; // node cost

      if ( idNear == idMin )
        continue; 

      Eigen::Vector3d posNear = posNds_.row(idNear);
 
      double potCstN = cstNew + cstL;
      if( potCstN < cstN )
      {
        //cstN = potCstN;
        modify_node(idNear, posNear, potCstN, idNew);

        break; // check if any solution is fine or the least has to be found
      }
    }

  } // end while

  //print_tree();
  //plot_tree();

}

// ***************************************************************************
Eigen::Vector3d rrt::rand_pos(double* min, double* max)
{
  std::mt19937 gen(std::chrono::system_clock::now().time_since_epoch().count());

  Eigen::Vector3d pt;

  std::uniform_real_distribution<double> dist_x(min[0], max[0]);
  pt(0) = dist_x(gen);

  std::uniform_real_distribution<double> dist_y(min[1], max[1]);
  pt(1) = dist_y(gen);

  std::uniform_real_distribution<double> dist_z(min[2], max[2]);
  pt(2) = dist_z(gen);

  return pt;
/*
  srand (time(NULL));

  Eigen::Vector3d pt;

  pt(0) = min[0] + (double)rand() / RAND_MAX * (max[0] - min[0]);
  pt(1) = min[1] + (double)rand() / RAND_MAX * (max[1] - min[1]);
  pt(2) = min[2] + (double)rand() / RAND_MAX * (max[2] - min[2]);

  return pt;
*/
}

// ***************************************************************************
int rrt::find_nearest(Eigen::Vector3d posRand)
{
  Eigen::Index minIndx;
  ( posNds_.topRows(actNds_).rowwise() - posRand.transpose() ).rowwise().squaredNorm().minCoeff(&minIndx);

  return minIndx;
}

// ***************************************************************************
void rrt::find_near(Eigen::Vector3d posNew, double rad)
{
  nearNds_.resize(0);

  near_node nearNd;
  
  for (int i=0; i<actNds_; i++)
  {
    nearNd.cstLnk = ( posNds_.row(i) - posNew.transpose() ).norm();

    if ( nearNd.cstLnk > rad )
      continue;
    if ( u_coll( posNds_.row(i), posNew ) )
      continue;

    nearNd.cstNd = cstNds_(i);
    nearNd.id = i;

    nearNds_.push_back(nearNd);
  }

  // ^ the orders can be reversed if col checks are less expensive than norms
}

// ***************************************************************************
Eigen::Vector3d rrt::steer(Eigen::Vector3d posFrom, Eigen::Vector3d posTowards, double delDist)
{
  Eigen::Vector3d vec = posTowards - posFrom;
  Eigen::Vector3d uVec = vec.normalized();

  if(vec == uVec) // if norm is zero, *this is unchanged
  {
    uVec = Eigen::Vector3d::UnitX();
    std::cout << "WARN: q_rand is very close to q_nearest, choosing random direction" << std::endl;
  }

  return posFrom + delDist * uVec;
}

// ***************************************************************************
bool rrt::u_coll(Eigen::Vector3d pos1, Eigen::Vector3d pos2)
{
  double delLambda = 0.2;

  double lambda = 0;
  Eigen::Vector3d pos;
  while(lambda <= 1)
  {
    pos = (1-lambda)*pos1 + lambda*pos2; 

    if ( u_coll_octomap(pos, radRob_, octDist_) )
      return true;

    lambda += delLambda;
  }

  return false;
}

// ***************************************************************************
bool rrt::u_coll_octomap(Eigen::Vector3d pos, double radRob, DynamicEDTOctomap* octDist)
{
  //std::cout << "Coll check for position" << pos << std::endl;
  double distObs = octDist->getDistance ( octomap::point3d( pos(0), pos(1), pos(2) ) );
  //std::cout << "Distance from the map" << distObs << std::endl;

  if ( distObs == DynamicEDTOctomap::distanceValue_Error )
    return false;

  if ( distObs <= radRob )
    return true;

  return false;
}

// ***************************************************************************
rrt::~rrt()
{
}

// ***************************************************************************
void rrt::update_oct_dist(DynamicEDTOctomap* octDist)
{
  octDist_ = octDist;
}

// ***************************************************************************
void rrt::print_near()
{
  std::cout << "<<<<<<<<<<<< Near Nodes >>>>>>>>>>>>>>>" << std::endl;

  for (int i=0; i<nearNds_.size(); i++)
  {
    std::cout << "Id: " << nearNds_[i].id << " Node Cost: " << nearNds_[i].cstNd << " Line Cost: " << nearNds_[i].cstLnk << std::endl;
  }

  std::cout << "<<<<<<<<<<<< ......... >>>>>>>>>>>>>>>" << std::endl;
}

// ***************************************************************************
void rrt::print_tree()
{
  std::cout << "<<<<<<<<<<<< Current Tree >>>>>>>>>>>>>>>" << std::endl;

  for (int i=0; i<actNds_; i++)
  {
    std::cout << "Id: " << i << " Pos: " << posNds_.row(i) << " Parent: " << idPrnts_[i] << " Node Cost: " << cstNds_(i) << std::endl;
  }

  std::cout << "<<<<<<<<<<<< ........... >>>>>>>>>>>>>>>" << std::endl;
}
// ***************************************************************************
void rrt::plot_tree()
{
  std::vector<double> vecX(2);
  std::vector<double> vecY(2);
  for (int i=0; i<actNds_; i++)
  {
    vecX[0] = posNds_(idPrnts_[i],0); vecX[1] = posNds_(i,0);
    vecY[0] = posNds_(idPrnts_[i],1); vecY[1] = posNds_(i,1);

    matplotlibcpp::plot(vecX,vecY);
  }

  //matplotlibcpp::xlim(minBnds_[0], maxBnds_[0]);
	//matplotlibcpp::ylim(minBnds_[1], maxBnds_[1]);
  matplotlibcpp::xlim(0, 6);
	matplotlibcpp::ylim(-6, 6);
  matplotlibcpp::pause(0.1);

  getchar();

  //double* arrX = posNds_.col(0).head(actNds_).data();
  //std::vector<double> vecX(arrX, arrX+actNds_);

  //double* arrY = posNds_.col(1).head(actNds_).data();
  //std::vector<double> vecY(arrY, arrY+actNds_);

  //matplotlibcpp::plot(vecX,vecY);
	
}
// ***************************************************************************
// ***************************************************************************



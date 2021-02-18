#include "rrt.h"

// ***************************************************************************
rrt::rrt(int nNodes, const std::vector<double>& minBnds, const std::vector<double>& maxBnds, double radNear, double delDist, double radRob, double succRad, int failItr, octomap_man* octMan)
{
  posNds_.resize(nNodes,3);
  cstNds_.resize(nNodes);
  idPrnts_.resize(nNodes);

  minBnds_[0] = minBnds[0];
  minBnds_[1] = minBnds[1];
  minBnds_[2] = minBnds[2];

  maxBnds_[0] = maxBnds[0];
  maxBnds_[1] = maxBnds[1];
  maxBnds_[2] = maxBnds[2];

  radNear_ = radNear;
  nearNds_.reserve(nNodes);

  delDist_ = delDist;
  nNodes_ = nNodes;

  radRob_ = radRob;
  succRad_ = succRad;

  failItr_ = failItr;

  octMan_ = octMan;
}

// ***************************************************************************
void rrt::set_bounds(const Eigen::Vector3d& minBnds, const Eigen::Vector3d& maxBnds)
{
  minBnds_[0] = minBnds(0); minBnds_[1] = minBnds(1); minBnds_[2] = minBnds(2);
  maxBnds_[0] = maxBnds(0); maxBnds_[1] = maxBnds(1); maxBnds_[2] = maxBnds(2);
}

// ***************************************************************************
void rrt::clear()
{
  //std::cout << "Clearing" << std::endl;
  actNds_ = 0;
  //idPrnts_.resize(0);
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
  //disp(actNds_, "Active Nodes");
  //disp(idLeaf, "Leaf Id");
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
std::vector<int> rrt::get_leaves()
{
  std::vector<int> idLvs;
  //disp(idPrnts_, "Parent Ids");
  //disp(idPrnts_.size(), "Parent Ids Size");
  for(int id=0; id<actNds_; id++)
  {
    if(std::find (idPrnts_.begin(), idPrnts_.end(), id) == idPrnts_.end())
      idLvs.push_back(id);
  }

  return idLvs;
}

// ***************************************************************************
void rrt::build(Eigen::Vector3d posRoot)
{
  build(posRoot, posRoot);
}

// ***************************************************************************
int rrt::build(const Eigen::Vector3d posRoot, const Eigen::Vector3d posGoal)
{
  posRoot_ = posRoot;

  //std::cout << "Building tree" << std::endl;
  clear();
  add_node(posRoot, 0, 0);

  //std::cout << "Added root node at position " << posRoot << std::endl;

  int itr = 0;
  while (actNds_ < nNodes_)
  {
    //print_tree();

    Eigen::Vector3d posRand;
    if (posRoot != posGoal)
      posRand = rand_pos(minBnds_, maxBnds_, posGoal);
    else
      posRand = rand_pos(minBnds_, maxBnds_);
   // std::cout << "Drawn random node at position " << posRand.transpose() << std::endl;

    int idNearest = find_nearest(posRand);
    Eigen::Vector3d posNearest = posNds_.row(idNearest);
   // std::cout << "Nearest node found at position " << posNearest.transpose() << std::endl;

    Eigen::Vector3d posNew = steer(posNearest, posRand, delDist_);
   // std::cout << "New node at position " << posNew.transpose() << std::endl;

    //getchar();

    itr++;
    if (itr > failItr_)
    {
      std::cout << "No solution found in max iterations!" << std::endl;
      break;
    }
    if ( octMan_->u_coll(posNearest, posNew) )
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

    if (posRoot != posGoal && (posNds_.row(actNds_-1).transpose() - posGoal).squaredNorm() < pow(succRad_,2) ) // assuming succRad = radRob_*4
      return actNds_-1;

  } // end while

  //print_tree();
  //plot_tree();
  return 0;
}

// ***************************************************************************
Eigen::Vector3d rrt::rand_pos(double* min, double* max, Eigen::Vector3d posGoal)
{
  const double epsilon = 0.95;

  std::mt19937 gen(std::chrono::system_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<double> dist(0, 1);

  if (dist(gen) >= epsilon)
    return posGoal;
  else
    return rand_pos(min, max);
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
  ( posNds_.topRows(actNds_).rowwise() - posRand.transpose() ).rowwise().norm().minCoeff(&minIndx);

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
    if ( octMan_->u_coll( posNds_.row(i), posNew ) )
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
rrt::~rrt()
{
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
  matplotlibcpp::show();

  std::vector<double> vecX(2);
  std::vector<double> vecY(2);

  for (int i=0; i<actNds_; i++)
  {
    vecX[0] = posNds_(idPrnts_[i],0); vecX[1] = posNds_(i,0);
    vecY[0] = posNds_(idPrnts_[i],1); vecY[1] = posNds_(i,1);

    matplotlibcpp::plot(vecX,vecY, "b--");
  }

  matplotlibcpp::xlim(minBnds_[0], maxBnds_[0]);
	matplotlibcpp::ylim(minBnds_[1], maxBnds_[1]);

  matplotlibcpp::show(false);
	
}
// ***************************************************************************
void rrt::plot_path(Eigen::MatrixXd path)
{
  std::vector<double> vecX(2);
  std::vector<double> vecY(2);
  for (int i=0; i<(path.rows()-1); i++)
  {
    vecX[0] = path(i,0); vecX[1] = path(i+1,0);
    vecY[0] = path(i,1); vecY[1] = path(i+1,1);

    matplotlibcpp::plot(vecX,vecY, "r");
  }
  matplotlibcpp::show(false);
}
// ***************************************************************************
void rrt::publish_viz(ros::Publisher& vizPub, std::string frameId, std::vector<int>& idLeaves)
{
  visualization_msgs::Marker tree;
  tree.header.frame_id = frameId;
  tree.header.stamp = ros::Time::now();
  tree.ns = "tree";
  tree.id = 0;
  tree.type = visualization_msgs::Marker::LINE_LIST;
  tree.action = visualization_msgs::Marker::ADD;
  tree.pose.orientation.w = 1;
  geometry_msgs::Vector3 scale;
	scale.x = 0.05;
	scale.y = 0.05;
	scale.z = 0.05;
	tree.scale = scale;
  tree.color.r = 0;
  tree.color.g = 0;
  tree.color.b = 1.0;
  tree.color.a = 1.0;

  std_msgs::ColorRGBA color;

  for(int i=0; i<idLeaves.size(); i++)
  {
    Eigen::MatrixXd path = get_path(idLeaves[i]);
    for(int j=0; j<(path.rows()-1); j++)
    {
      geometry_msgs::Point vertex;
      vertex.x = path(j,0);
      vertex.y = path(j,1);
      vertex.z = path(j,2);

      color.r = 0; color.g = 0; color.b = 1.0; color.a = 1.0;
      tree.colors.push_back(color);

      tree.points.push_back(vertex);
      vertex.x = path(j+1,0);
      vertex.y = path(j+1,1);
      vertex.z = path(j+1,2);
      tree.points.push_back(vertex);

      color.r = 0; color.g = 0; color.b = 1.0; color.a = 1.0;
      tree.colors.push_back(color);
    }
  }

  visualization_msgs::MarkerArray vizMsg;
  vizMsg.markers.push_back(tree);
  
  vizPub.publish(vizMsg);
}

// ***************************************************************************
double rrt::get_del_dist()
{
  return delDist_;
}


// ***************************************************************************



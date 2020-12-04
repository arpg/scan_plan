#ifndef DISP_H
#define DISP_H

#include <vector>
#include <string>
#include <iostream>
// ***************************************************************************
// ***************************************************************************
template <class T>
void disp(std::vector<T>& vecIn, std::string name="-")
{
  std::cout << name << ": ";
 
  if (vecIn.size() == 0)
    return;

  std::cout << vecIn[0];
  for(int i=1; i<vecIn.size(); i++)
    std::cout << ", " << vecIn[i];
  std::cout << std::endl;
}

// ***************************************************************************
template <class T>
void disp(T* arrIn, int size, std::string name="-")
{
  std::cout << name << ": ";
 
  if (size == 0)
    return;

  std::cout << arrIn[0];
  for(int i=1; i<size; i++)
    std::cout << ", " << arrIn[i];
  std::cout << std::endl;
}

// ***************************************************************************
template <class T>
void disp(T varIn, std::string name="-")
{
  std::cout << name << ": " << varIn << std::endl;
}

#endif

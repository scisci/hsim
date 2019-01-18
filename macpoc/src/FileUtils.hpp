//
//  FileUtils.hpp
//  hsimall
//
//  Created by z on 1/16/19.
//

#ifndef FileUtils_h
#define FileUtils_h

#include <functional>
#include <string>

class TestFile {
public:
  static void OpenDirectory(std::function<void(std::string)> cb);
};


#endif /* FileUtils_h */

//
//  FileUtils.m
//  macpoc
//
//  Created by z on 1/16/19.
//

#import <Foundation/Foundation.h>

#import "FileUtils.hpp"
#import "SandboxFileManager.h"


void TestFile::OpenDirectory(std::function<void(std::string)> cb)
{
  SandboxFileManager *fm = [[SandboxFileManager alloc] initWithPrefix:@"mypref"];
  [fm openUrl:nil withCompletion:^(URLResource *resource) {
    if (resource == nil) {
      cb(std::string(""));
      return;
    }

    NSString* path = resource.url.path;
    cb(std::string([path cStringUsingEncoding:NSUTF8StringEncoding]));
  }];
}

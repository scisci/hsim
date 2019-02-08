//
//  FileUtils.m
//  macpoc
//
//  Created by z on 1/16/19.
//

#import <Foundation/Foundation.h>

#import "FileUtils.hpp"
#import "SandboxFileManager.h"


void OpenWithOptions(OpenOptions *options, std::function<void(std::string)> cb)
{
  SandboxFileManager *fm = [[SandboxFileManager alloc] initWithPrefix:@"mypref"];
  
  [fm openUrl:nil withOptions: options withCompletion:^(URLResource *resource) {
    if (resource == nil) {
      cb(std::string(""));
      return;
    }

    NSString* path = resource.url.path;
    cb(std::string([path cStringUsingEncoding:NSUTF8StringEncoding]));
  }];
}

void TestFile::OpenDirectory(std::function<void(std::string)> cb)
{
  OpenOptions *options = [[OpenOptions alloc] init];
  options.canChooseDirectories = YES;
  options.canChooseFiles = NO;
  OpenWithOptions(options, cb);
}

void TestFile::OpenFile(std::function<void(std::string)> cb)
{
  OpenOptions *options = [[OpenOptions alloc] init];
  options.canChooseDirectories = NO;
  options.canChooseFiles = YES;
  OpenWithOptions(options, cb);
}

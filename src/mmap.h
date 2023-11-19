#ifndef _MMAP_H
#define	_MMAP_H

#ifdef _WIN32
#include <stdio.h>
#include <Windows.h>

HANDLE hFile;
HANDLE hMap;
LPVOID lpBasePtr;

// much faster than getline
// https://stackoverflow.com/questions/68368291/mapping-files-into-virtual-memory-in-c-on-windows
int map_file(const char *fname, size_t &length) // loadfile_mmap
{
//  TCHAR *lpFileName = TEXT("hello.txt");
//  const TCHAR *lpFileName = fname;
  LARGE_INTEGER liFileSize;

  hFile = CreateFile(fname,
    GENERIC_READ,                          // dwDesiredAccess
    0,                                     // dwShareMode
    NULL,                                  // lpSecurityAttributes
    OPEN_EXISTING,                         // dwCreationDisposition
    FILE_ATTRIBUTE_NORMAL,                 // dwFlagsAndAttributes
    0);                                    // hTemplateFile
  if (hFile == INVALID_HANDLE_VALUE) {
    fprintf(stderr, "CreateFile failed with error %d\n", GetLastError());
//    return 1;
  }

  if (!GetFileSizeEx(hFile, &liFileSize)) {
    fprintf(stderr, "GetFileSize failed with error %d\n", GetLastError());
    CloseHandle(hFile);
//    return 1;
  }
  length = (size_t)liFileSize.QuadPart;

  if (liFileSize.QuadPart == 0) {
    fprintf(stderr, "File is empty\n");
    CloseHandle(hFile);
//    return 1;
  }

  hMap = CreateFileMapping(
    hFile,
    NULL,                          // Mapping attributes
    PAGE_READONLY,                 // Protection flags
    0,                             // MaximumSizeHigh
    0,                             // MaximumSizeLow
    NULL);                         // Name
  if (hMap == 0) {
    fprintf(stderr, "CreateFileMapping failed with error %d\n", GetLastError());
    CloseHandle(hFile);
//    return 1;
  }

  lpBasePtr = MapViewOfFile(
    hMap,
    FILE_MAP_READ,         // dwDesiredAccess
    0,                     // dwFileOffsetHigh
    0,                     // dwFileOffsetLow
    0);                    // dwNumberOfBytesToMap
  if (lpBasePtr == NULL) {
    fprintf(stderr, "MapViewOfFile failed with error %d\n", GetLastError());
    CloseHandle(hMap);
    CloseHandle(hFile);
    return 1;
  }

  // Display file content as ASCII charaters
/* nah...
  std::vector<std::string> s_header;
  LONGLONG i = liFileSize.QuadPart;
  char *ptr = (char *)lpBasePtr;
  while (i-- > 0) {
    fputc(*ptr++, stdout);
  }
  */


  // TODO close fd at some point in time, call munmap(...)
//  UnmapViewOfFile(lpBasePtr);
//  CloseHandle(hMap);
//  CloseHandle(hFile);

  printf("\nmapping done.\n");
  return 0;
}

void close_mapfile() {
  UnmapViewOfFile(lpBasePtr);
  CloseHandle(hMap);
  CloseHandle(hFile);
}

#else
// Linux, Mac etc.
const char* map_file(const char* fname, size_t& length)
{
  int fd = open(fname, O_RDONLY);
  if (fd == -1)
    handle_error("open");

  // obtain file size
  struct stat sb;
  if (fstat(fd, &sb) == -1)
    handle_error("fstat");

  length = sb.st_size;

  const char* addr = static_cast<const char*>(mmap(NULL, length, PROT_READ, MAP_PRIVATE, fd, 0u));
  if (addr == MAP_FAILED)
    handle_error("mmap");

  // TODO close fd at some point in time, call munmap(...)
  return addr;
}
#endif

#endif // _MMAP_H

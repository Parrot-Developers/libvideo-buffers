# libvideo-buffers - Video buffers library

libvideo-buffers is a C library to handle video buffers, including allocation,
reference counting, write-protection, multiple metadata, and user data.

The library also provides thread-safe buffer pools and buffer queues. Pools and
queues can be used with notifications on a pomp loop using pomp_evt.

## Implementations

Custom implementation for specific platforms or video libraries can be defined
by using callback functions. The following implementations are available:

* _generic_ (through _libvideo-buffers-generic_) using standard malloc/free

## Dependencies

The library depends on the following Alchemy modules:

* libulog
* libfutils
* libpomp

## Building

Building is activated by enabling _libvideo-buffers_ in the Alchemy build
configuration.

## Operation

### Threading model

All API functions are thread safe.

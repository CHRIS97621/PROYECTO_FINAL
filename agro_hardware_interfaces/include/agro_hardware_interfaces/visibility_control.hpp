
// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AGRO_HARDWARE_INTERFACES__VISIBILITY_CONTROL_H_
#define AGRO_HARDWARE_INTERFACES__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define AGRO_HARDWARE_INTERFACES_EXPORT __attribute__((dllexport))
#define AGRO_HARDWARE_INTERFACES_IMPORT __attribute__((dllimport))
#else
#define AGRO_HARDWARE_INTERFACES_EXPORT __declspec(dllexport)
#define AGRO_HARDWARE_INTERFACES_IMPORT __declspec(dllimport)
#endif
#ifdef AGRO_HARDWARE_INTERFACES_BUILDING_DLL
#define AGRO_HARDWARE_INTERFACES_PUBLIC AGRO_HARDWARE_INTERFACES_EXPORT
#else
#define AGRO_HARDWARE_INTERFACES_PUBLIC AGRO_HARDWARE_INTERFACES_IMPORT
#endif
#define AGRO_HARDWARE_INTERFACES_PUBLIC_TYPE AGRO_HARDWARE_INTERFACES_PUBLIC
#define AGRO_HARDWARE_INTERFACES_LOCAL
#else
#define AGRO_HARDWARE_INTERFACES_EXPORT __attribute__((visibility("default")))
#define AGRO_HARDWARE_INTERFACES_IMPORT
#if __GNUC__ >= 4
#define AGRO_HARDWARE_INTERFACES_PUBLIC __attribute__((visibility("default")))
#define AGRO_HARDWARE_INTERFACES_LOCAL __attribute__((visibility("hidden")))
#else
#define AGRO_HARDWARE_INTERFACES_PUBLIC
#define AGRO_HARDWARE_INTERFACES_LOCAL
#endif
#define AGRO_HARDWARE_INTERFACES_PUBLIC_TYPE
#endif

#endif  // AGRO_HARDWARE_INTERFACES__VISIBILITY_CONTROL_H_
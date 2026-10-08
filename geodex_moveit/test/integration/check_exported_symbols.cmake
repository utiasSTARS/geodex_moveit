# Copyright 2026 Space and Terrestrial Autonomous Robotic Systems (STARS) Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Fails when a library exports an OMPL, Eigen or geodex symbol. Two OMPL builds with one soname
# and three Eigen versions can share a MoveIt process.
foreach(lib IN LISTS LIBRARIES)
  execute_process(COMMAND nm -D --defined-only -C ${lib}
    OUTPUT_VARIABLE symbols RESULT_VARIABLE rc)
  if(NOT rc EQUAL 0)
    message(FATAL_ERROR "nm failed on ${lib}")
  endif()
  string(REPLACE "geodex_moveit::" "" symbols "${symbols}")
  string(REGEX MATCHALL "[^\n]*(ompl::|Eigen::|geodex::)[^\n]*" leaked "${symbols}")
  list(LENGTH leaked n)
  if(n GREATER 0)
    list(SUBLIST leaked 0 10 first)
    string(REPLACE ";" "\n" first "${first}")
    message(FATAL_ERROR "${lib} exports ${n} OMPL, Eigen or geodex symbols, for example\n${first}")
  endif()
  message(STATUS "${lib}: no OMPL, Eigen or geodex symbol exported")
endforeach()

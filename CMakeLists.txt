################################################################################
#                                                                              #
#   Copyright (c) 2007-2009: Peter Schregle,                                   #
#   All rights reserved.                                                       #
#                                                                              #
#   This file is part of the Fixed Point Math Library.                         #
#                                                                              #
#   Redistribution of the Fixed Point Math Library and use in source and       #
#   binary forms, with or without modification, are permitted provided that    #
#   the following conditions are met:                                          #
#   1. Redistributions of source code must retain the above copyright notice,  #
#      this list of conditions and the following disclaimer.                   #
#   2. Redistributions in binary form must reproduce the above copyright       #
#      notice, this list of conditions and the following disclaimer in the     #
#      documentation and/or other materials provided with the distribution.    #
#   3. Neither the name of Peter Schregle nor the names of other contributors  #
#      may be used to endorse or promote products derived from this software   #
#      without specific prior written permission.                              #
#                                                                              #
#   THIS SOFTWARE IS PROVIDED BY PETER SCHREGLE AND CONTRIBUTORS 'AS IS' AND   #
#   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE      #
#   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE #
#   ARE DISCLAIMED. IN NO EVENT SHALL PETER SCHREGLE OR CONTRIBUTORS BE LIABLE #
#   FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL #
#   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS    #
#   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)      #
#   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,        #
#   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN   #
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE            #
#   POSSIBILITY OF SUCH DAMAGE.                                                #
#                                                                              #
################################################################################

cmake_minimum_required (VERSION 3.5.1)
project (FPMATH)

set (FPML_VERSION_MAJOR 0)
set (FPML_VERSION_MINOR 2)
set (FPML_NAME "Fixed Point Math Library")

include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}/ext/ex_type_traits/include")

add_subdirectory(ext/ex_type_traits)
add_subdirectory(test)
add_subdirectory(doc)

/*
 *    This file is part of CasADi.
 *
 *    CasADi -- A symbolic framework for dynamic optimization.
 *    Copyright (C) 2010-2014 Joel Andersson, Joris Gillis, Moritz Diehl,
 *                            K.U. Leuven. All rights reserved.
 *    Copyright (C) 2011-2014 Greg Horn
 *
 *    CasADi is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    CasADi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with CasADi; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


#include "casadi_interrupt.hpp"

#ifdef CASADI_WITH_THREAD
#ifdef CASADI_WITH_THREAD_MINGW
#include <mingw.thread.h>
#else // CASADI_WITH_THREAD_MINGW
#include <thread>
#endif // CASADI_WITH_THREAD_MINGW
#endif //CASADI_WITH_THREAD

namespace casadi {

  bool (*InterruptHandler::checkInterrupted)() =
    InterruptHandler::checkInterruptedDefault;

  void (*InterruptHandler::clearInterrupted)() =
    InterruptHandler::clearInterruptedDefault;

  bool InterruptHandler::is_main_thread() {
#ifdef CASADI_WITH_THREAD
    static std::thread::id main = std::this_thread::get_id();
    return std::this_thread::get_id()==main;
#else //CASADI_WITH_THREAD
    return true;
#endif //CASADI_WITH_THREAD
  }

} // namespace casadi

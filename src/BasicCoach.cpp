/*
Copyright (c) 2000-2003, Jelle Kok, University of Amsterdam
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the University of Amsterdam nor the names of its
contributors may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*! \file BasicCoach.cpp
<pre>
<b>File:</b>          BasicCoach.cpp
<b>Project:</b>       Robocup Soccer Simulation Team: UvA Trilearn
<b>Authors:</b>       Jelle Kok
<b>Created:</b>       03/03/2001
<b>Last Revision:</b> $ID$
<b>Contents:</b>      This file contains the class definitions for the
                      BasicCoach which contains the main structure for the 
                      coach.
<hr size=2>
<h2><b>Changes</b></h2>
<b>Date</b>             <b>Author</b>          <b>Comment</b>
03/03/2001        Jelle Kok       Initial version created
</pre>
*/

#include"BasicCoach.h"
#include"Parse.h"
#ifdef WIN32
  #include <windows.h>
#else
  #include <sys/poll.h>
#endif

#include <iostream>
#include <sstream>
#include "hungarian.h"  /* the hungarian method for dynamic role assignment */


extern Logger Log; /*!< This is a reference to the Logger to write loginfo to*/

/*!This is the constructor for the BasicCoach class and contains the
   arguments that are used to initialize a coach.
   \param act ActHandler to which the actions can be sent
   \param wm WorldModel which information is used to determine action
   \param ss ServerSettings that contain parameters used by the server
   \param strTeamName team name of this player
   \param dVersion version this basiccoach corresponds to
   \param isTrainer indicates whether the coach is a trainer (offline coach)
          or an online coach (used during the match). */
BasicCoach::BasicCoach( ActHandler* act, WorldModel *wm, ServerSettings *ss,
      char* strTeamName, double dVersion, bool isTrainer )

{
    char str[MAX_MSG];

    ACT       = act;
    WM        = wm;
    SS        = ss;
    bContLoop = true;
    WM->setTeamName( strTeamName );

    if( !isTrainer )
        sprintf( str, "(init %s (version %f))", strTeamName, dVersion );
    else
        sprintf( str, "(init (version %f))", dVersion );

    ACT->sendMessage( str );
}

BasicCoach::~BasicCoach( )
{
}

/*! This method is the main loop of the coach. All sequence of actions are
    located in this method. */
void BasicCoach::mainLoopNormal( )
{
#ifdef WIN32
    Sleep( 1000 );
#else
    poll( 0, 0, 1000 );
#endif

    bool bSubstituted   = false;
    ACT->sendMessageDirect( "(eye on)" );

#ifdef WIN32
    Sleep( 1000 );
#else
    poll( 0, 0, 1000 );
#endif

    timeLastRoleAssign = WM->getCurrentTime();

    while( WM->getPlayMode() != PM_TIME_OVER  && bContLoop )
    {

        if(  WM->waitForNewInformation() == false )
        {
            printf( "Shutting down coach\n" );
            bContLoop = false;
        }

        if (WM->getPlayMode() == PM_PLAY_ON &&
                WM->getCurrentTime() - timeLastRoleAssign >= 100)
        {

            /*
             * The last time we sent a role assignment to the players is more
             * than 99 game cycles away, therefore it's time to recalculate
             */

            timeLastRoleAssign = WM->getCurrentTime();


            /*
             * Retrieves the positions and home positions of all players of our
             * team.  They are used to calculate the distance matrix, which is
             * used in the hungarian method to determine the ideal role
             * assignment.
             */
            VecPosition playerPositions[PLAYER_CNT];
            VecPosition homePositions[PLAYER_CNT];
            for (int i = 0; i < PLAYER_CNT; i++)
            {
                /*
                 * ObjectT is an enumeration in SoccerTypes.h, that contains all
                 * object types TriLearn knows about.  To retrieve the position
                 * of a player, the corresponding OBJECT_TEAMMATE_n enum
                 * constant has to be used with getGlobalPositionLastSee, where
                 * n is a number between 1 and 11.
                 *
                 * To get to the correct enum constant, the index i
                 * (ranges from 0 to 10) is added to the int representation of
                 * OBJECT_TEAMMATE_1, which is then casted back to ObjectT.
                 */
                ObjectT playerType = (ObjectT) (i + (int)(OBJECT_TEAMMATE_1));
                playerPositions[i] = WM->getGlobalPositionLastSee(playerType);

                /* Home position indeces range from 0 to 10 */
                homePositions[i] = WM->getHomePos(i);
            }

            // cout << "[DEBUG] player positions:" << endl;
            // for (int i = 0; i < PLAYER_CNT; i++)
            //     cout << "[DEBUG]    player " << i << ": "
            //          << playerPositions[i].getX() << " "
            //          << playerPositions[i].getY() << endl;

            // cout << "[DEBUG] home positions:" << endl;
            // for (int i = 0; i < PLAYER_CNT; i++)
            //     cout << "[DEBUG]    home " << i << ": "
            //          << homePositions[i].getX() << " "
            //          << homePositions[i].getY() << endl;

            /*
             * Calculates the distance from each player to each home position
             * and saves the result in a PLAYER_CNT*PLAYER_CNT matrix.  Each
             * element (p, h) of the matrix is the distance between player p and
             * home position h.
             */
            int distances[PLAYER_CNT][PLAYER_CNT];
            for (int iPlayer = 0; iPlayer < PLAYER_CNT; ++iPlayer)
            {
                for (int iHomePos = 0; iHomePos < PLAYER_CNT; ++iHomePos)
                {
                    double distance = playerPositions[iPlayer]
                        .getDistanceTo(homePositions[iHomePos]);

                    /*
                     * The C-implementation of the hungarian method I'm using
                     * expects a matrix of ints.  To avoid a high loss of
                     * accuracy, I scale up the distance by 1000 and then
                     * discard the decimal places by casting to int.
                     *
                     * The hungarian method only inspects the distances relative
                     * to each other.  Therefore, scaling up does not change the
                     * result.
                     */
                    distances[iPlayer][iHomePos] = (int) (distance * 1000);
                }
            }

            /* Initializes the hungarian method implementation. */
            hungarian_t prob;
            hungarian_init( &prob           /* the algorithms internal data   */
                          , (int*)distances /* the cost/distance matrix       */
                          , PLAYER_CNT      /* height of the cost matrix      */
                          , PLAYER_CNT      /* width of the cost matrix       */
                          , HUNGARIAN_MIN );/* algorithm should minimize cost */

            /* Finds the optimal role assignment and saves it in prob.a */
            hungarian_solve(&prob);

            /* sends the calculated assignment to the players so that they can
             * adjust their positions */
            sendAssignment(prob.a, homePositions);

            /* releases all resources acquired for the hungarian method */
            hungarian_fini(&prob);
        }


        if( Log.isInLogLevel(  456 ) )
            WM->logObjectInformation( 456, OBJECT_ILLEGAL);
        if( SS->getSynchMode() == true )
            ACT->sendMessageDirect( "(done)" );
    }

    return;
}


void BasicCoach::sendAssignment(int *assignment, VecPosition homePositions[])
{
    /*
     * The CLang message we send to the players has the following format:
     *
     *     (say (advice (0 (true)
     *         (do our {<player-number>} (home (pt <x> <y>)))
     *         (do our {<player-number>} (home (pt <x> <y>)))
     *         ...)))
     *
     * where  <player-number>  is the trikot number of the player, the command
     *                         applies to.  It ranges from 1 to 11;
     *
     *        <x> and <y>      are the x and y coordinates of the new home
     *                         position of the player in a coordinate system
     *                         where (0, 0) is the center of the field.
     *
     */

    stringstream ss;
    ss << "(say (advice (0 (true) ";
    for (int i = 0; i < PLAYER_CNT; ++i)
    {
        /* "assignment" is simply an array of indices into homePositions */
        int index = assignment[i];
        int x = (int) homePositions[index].getX();
        int y = (int) homePositions[index].getY();

        ss << "(do our {" << (i+1)
           << "} (home (pt "
           << x << " " << y << ")))";
    }
    ss << ")))";

    /* ACT->sendMessage expects a C-style (char *), and not an std::string.
     * Therefore, the std::string is retrieved from the std::stringstream here,
     * and then converted to a C-string. */
    char *message = (char *)ss.str().c_str();

    cout << message;
    ACT->sendMessage( message );
}

/*! This method substitutes one player to the given player type and sends
    this command (using the ActHandler) to the soccer server. */
void BasicCoach::substitutePlayer( int iPlayer, int iPlayerType )
{
    SoccerCommand soc;
    soc.makeCommand( CMD_CHANGEPLAYER, (double)iPlayer, (double)iPlayerType );
    ACT->sendCommandDirect( soc );
    cerr << "coachmsg: changed player " << iPlayer << " to type " << iPlayerType
         << endl;
}


#ifdef WIN32
DWORD WINAPI stdin_callback( LPVOID v )
#else
void* stdin_callback( void * v )
#endif
{
    Log.log( 1, "Starting to listen for user input" );
    BasicCoach* bc = (BasicCoach*)v;
    bc->handleStdin();
    return NULL;
}

/*!This method listens for input from the keyboard and when it receives this
   input it converts this input to the associated action. See
   showStringCommands for the possible options. This method is used together
   with the SenseHandler class that sends an alarm to indicate that a new
   command can be sent. This conflicts with the method in this method that
   listens for the user input (fgets) on Linux systems (on Solaris this isn't a
   problem). The only known method is to use the flag SA_RESTART with this
   alarm function, but that does not seem to work under Linux. If each time
   the alarm is sent, this gets function unblocks, it will cause major
   performance problems. This function should not be called when a whole match
   is played! */
void BasicCoach::handleStdin( )
{
    char buf[MAX_MSG];

    while( bContLoop )
    {
#ifdef WIN32
        cin.getline( buf, MAX_MSG );
#else
        fgets( buf, MAX_MSG, stdin ); // does unblock with signal !!!!!
#endif
        printf( "after fgets: %s\n", buf );
        executeStringCommand( buf );
    }
}

/*!This method prints the possible commands that can be entered by the user.
   The whole name can be entered to perform the corresponding command, but
   normally only the first character is sufficient. This is indicated by
   putting brackets around the part of the command that is not needed.
   \param out output stream to which the possible commands are printed */
void BasicCoach::showStringCommands( ostream& out )
{
    out << "Basic commands:"        << endl <<
           " m(ove) player_nr x y"  << endl <<
           " q(uit)"                << endl;
}

/*!This method executes the command that is entered by the user. For the
   possible command look at the method showStringCommands.
   \param str string that is entered by the user
   \return true when command could be executed, false otherwise */
bool BasicCoach::executeStringCommand( char *str)
{
    switch( str[0] )
    {
        case 'm':                               // move
            sprintf( str, "(move %d %f %f)", Parse::parseFirstInt( &str ),
                    Parse::parseFirstDouble( &str ),
                    Parse::parseFirstDouble( &str ) );
            break;
        case 'q':                             // quit
            bContLoop = false;
            return true;
        default:                             // default: send entered string
            ;
    }
    printf( "send: %s\n", str );
    ACT->sendMessage( str );
    return true;
}

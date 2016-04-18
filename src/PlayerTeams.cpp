/*
Copyright (c) 2000-2003, Jelle Kok, University of Amsterdam
Updated by Samira Karimzadeh 02/09/2009
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

/*! \file PlayerTeams.cpp
<pre>
<b>File:</b>          PlayerTest.cpp
<b>Project:</b>       Robocup Soccer Simulation Team: UvA Trilearn
<b>Authors:</b>       Jelle Kok
<b>Created:</b>       10/12/2000
<b>Last Revision:</b> $ID$
<b>Contents:</b>      This file contains the class definitions for the
                      Player that are used to test the teams' high level
                      strategy.
<hr size=2>
<h2><b>Changes</b></h2>
<b>Date</b>             <b>Author</b>          <b>Comment</b>
10/12/2000        Jelle Kok       Initial version created
</pre>
*/

#include "Player.h"

/*!This method is the first complete simple team and defines the actions taken
   by all the players on the field (excluding the goalie). It is based on the
   high-level actions taken by the simple team FC Portugal that it released in
   2000. The players do the following:
   - if ball is kickable
       kick ball to goal (random corner of goal)
   - else if i am fastest player to ball
       intercept the ball
   - else
       move to strategic position based on your home position and pos ball */
SoccerCommand Player::deMeer5(  )
{

  SoccerCommand soc(CMD_ILLEGAL);

  VecPosition   posAgent = WM->getAgentGlobalPosition();
  VecPosition   posBall  = WM->getBallPos();
  int           iTmp;

  if( WM->isBeforeKickOff( ) )
  {
    if( WM->isKickOffUs( ) && WM->getPlayerNumber() == 9 ) // 9 takes kick
    {
      if( WM->isBallKickable() )
      {
            VecPosition posGoal( PITCH_LENGTH/2.0,
                             (-1 + 2*(WM->getCurrentCycle()%2)) *
                             0.4 * SS->getGoalWidth() );

        int Number=WM->getPlayerNumber();
        VecPosition a(0,0);

        if( isKickable(a) )
        {
        soc = kickTo( a, SS->getBallSpeedMax() ); // kick maximal
        }
        else
        {
        int x=Esuitable();
        double dist=0.5;
        if(x!=OBJECT_TEAMMATE_UNKNOWN)
        {
            soc = directPass(WM->getGlobalPosition((ObjectT)(x)),PASS_FAST);
        }
        else
        {
            Circle c( posAgent , 2.5 );
            AngDeg angle;
            if( (isDribbleFast(angle)) &&
                ( !WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS , c) ) )
            {
                soc=dribble(angle,DRIBBLE_FAST);
            }

            else
            {
            if( (coneDribble(angle)) &&
            ( !WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS , c) ) )
            {
                soc=dribble(angle,DRIBBLE_WITHBALL);

            }
            else
            {
                VecPosition P3( 52.5 , 6 );
                VecPosition P4( 52.5 , -6 );
                VecPosition opponentGoalie = WM->getGlobalPosition((ObjectT)(73));

                if( opponentGoalie.getDistanceTo(P3) >=
                opponentGoalie.getDistanceTo(P4) )
                {
                soc = kickTo( P3, SS->getBallSpeedMax() ); // kick maximal
                }
                else
                {
                soc = kickTo( P4, SS->getBallSpeedMax() ); // kick maximal
                }
                //soc = kickTo( posGoal, SS->getBallSpeedMax() ); // kick maximal
            }
            }
        }

        }


        //***********************************************
       // soc = kickTo( posGoal, SS->getBallSpeedMax() ); // kick maximal
//        Log.log( 100, "take kick off" );
      }
      else
      {
        soc = intercept( false );
//        Log.log( 100, "move to ball to take kick-off" );
      }
      ACT->putCommandInQueue( soc );
      ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
      return soc;
    }
    if( formations->getFormation() != FT_INITIAL || // not in kickoff formation
        posAgent.getDistanceTo( WM->getStrategicPosition() ) > 2.0 )
    {
      formations->setFormation( FT_INITIAL );       // go to kick_off formation
      ACT->putCommandInQueue( soc=teleportToPos( WM->getStrategicPosition() ));
    }
    else                                            // else turn to center
    {
      ACT->putCommandInQueue( soc=turnBodyToPoint( VecPosition( 0, 0 ), 0 ) );
      ACT->putCommandInQueue( alignNeckWithBody( ) );
    }
  }
  else
  {
    formations->setFormation( FT_OFFENSIVE );
    soc.commandType = CMD_ILLEGAL;

    if( WM->getConfidence( OBJECT_BALL ) < PS->getBallConfThr() )
    {
      ACT->putCommandInQueue( soc = searchBall() );   // if ball pos unknown
      ACT->putCommandInQueue( alignNeckWithBody( ) ); // search for it
    }
    else if( WM->isBallKickable())                    // if kickable
    {
      VecPosition posGoal( PITCH_LENGTH/2.0,
              (-1 + 2*(WM->getCurrentCycle()%2)) * 0.4 * SS->getGoalWidth() );
      soc = kickTo( posGoal, SS->getBallSpeedMax() ); // kick maximal

      ACT->putCommandInQueue( soc );
      ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
//      Log.log( 100, "kick ball" );
    }
    else if( WM->getFastestInSetTo( OBJECT_SET_TEAMMATES, OBJECT_BALL, &iTmp )
              == WM->getAgentObjectType()  && !WM->isDeadBallThem() )
    {                                                // if fastest to ball
//      Log.log( 100, "I am fastest to ball; can get there in %d cycles", iTmp );
      soc = intercept( false );                      // intercept the ball

      if( soc.commandType == CMD_DASH &&             // if stamina low
          WM->getAgentStamina().getStamina() <
             SS->getRecoverDecThr()*SS->getStaminaMax()+200 )
      {
        soc.dPower = 30.0 * WM->getAgentStamina().getRecovery(); // dash slow
        ACT->putCommandInQueue( soc );
        ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
      }
      else                                           // if stamina high
      {
        ACT->putCommandInQueue( soc );               // dash as intended
        ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
      }
     }
     else if( posAgent.getDistanceTo(WM->getStrategicPosition()) >
                  1.5 + fabs(posAgent.getX()-posBall.getX())/10.0)
                                                  // if not near strategic pos
     {
       if( WM->getAgentStamina().getStamina() >     // if stamina high
                            SS->getRecoverDecThr()*SS->getStaminaMax()+800 )
       {
         soc = moveToPos(WM->getStrategicPosition(),
                         PS->getPlayerWhenToTurnAngle());
         ACT->putCommandInQueue( soc );            // move to strategic pos
         ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
       }
       else                                        // else watch ball
       {
         ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
         ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
       }
     }
     else if( fabs( WM->getRelativeAngle( OBJECT_BALL ) ) > 1.0 ) // watch ball
     {
       ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
       ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
     }
     else                                         // nothing to do
       ACT->putCommandInQueue( SoccerCommand(CMD_TURNNECK,0.0) );
   }
  return soc;
}

/*!This method is a simple goalie based on the goalie of the simple Team of
   FC Portugal. It defines a rectangle in its penalty area and moves to the
   position on this rectangle where the ball intersects if you make a line
   between the ball position and the center of the goal. If the ball can
   be intercepted in the own penalty area the ball is intercepted and catched.
*/
SoccerCommand Player::deMeer5_goalie(  )
{
  int i;
  SoccerCommand soc;
  VecPosition   posAgent = WM->getAgentGlobalPosition();
  AngDeg        angBody  = WM->getAgentGlobalBodyAngle();

  // define the top and bottom position of a rectangle in which keeper moves
  static const VecPosition posLeftTop( -PITCH_LENGTH/2.0 +
               0.7*PENALTY_AREA_LENGTH, -PENALTY_AREA_WIDTH/4.0 );
  static const VecPosition posRightTop( -PITCH_LENGTH/2.0 +
               0.7*PENALTY_AREA_LENGTH, +PENALTY_AREA_WIDTH/4.0 );

  // define the borders of this rectangle using the two points.
  static Line  lineFront = Line::makeLineFromTwoPoints(posLeftTop,posRightTop);
  static Line  lineLeft  = Line::makeLineFromTwoPoints(
                         VecPosition( -50.0, posLeftTop.getY()), posLeftTop );
  static Line  lineRight = Line::makeLineFromTwoPoints(
                         VecPosition( -50.0, posRightTop.getY()),posRightTop );


  if( WM->isBeforeKickOff( ) )
  {
    if( formations->getFormation() != FT_INITIAL || // not in kickoff formation
        posAgent.getDistanceTo( WM->getStrategicPosition() ) > 2.0 )
    {
      formations->setFormation( FT_INITIAL );       // go to kick_off formation
      ACT->putCommandInQueue( soc=teleportToPos(WM->getStrategicPosition()) );
    }
    else                                            // else turn to center
    {
      ACT->putCommandInQueue( soc = turnBodyToPoint( VecPosition( 0, 0 ), 0 ));
      ACT->putCommandInQueue( alignNeckWithBody( ) );
    }
    return soc;
  }

  if( WM->getConfidence( OBJECT_BALL ) < PS->getBallConfThr() )
  {                                                // confidence ball too  low
    ACT->putCommandInQueue( searchBall() );        // search ball
    ACT->putCommandInQueue( alignNeckWithBody( ) );
  }
  else if( WM->getPlayMode() == PM_PLAY_ON || WM->isFreeKickThem() ||
           WM->isCornerKickThem() )
  {
    if( WM->isBallCatchable() )
    {
      ACT->putCommandInQueue( soc = catchBall() );
      ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
    }
     else if( WM->isBallKickable() )
    {

       int temp=suitablePass1();
       soc = directPass(WM->getGlobalPosition((ObjectT)(temp)),PASS_FAST);
       ACT->putCommandInQueue( soc );
       ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );

    }
    else if( WM->isInOwnPenaltyArea( getInterceptionPointBall( &i, true ) ) &&
             WM->getFastestInSetTo( OBJECT_SET_PLAYERS, OBJECT_BALL, &i ) ==
                                               WM->getAgentObjectType() )
    {
      ACT->putCommandInQueue( soc = intercept( true ) );
      ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
    }
    else
    {
        VecPosition ball=WM->getGlobalPosition(OBJECT_BALL);
    if(ball.getX() >= ( (WM->getGlobalPosition(OBJECT_TEAMMATE_1).getX() ) ) )
    {
         //********* Goal Line ******
      soc=defendGoalLine(3);
      ACT->putCommandInQueue(soc);
         ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
        //**************************
     }
    else
    {
      // make line between own goal and the ball
            VecPosition posMyGoal = ( WM->getSide() == SIDE_LEFT )
             ? SoccerTypes::getGlobalPositionFlag(OBJECT_GOAL_L, SIDE_LEFT )
             : SoccerTypes::getGlobalPositionFlag(OBJECT_GOAL_R, SIDE_RIGHT);
            Line lineBall = Line::makeLineFromTwoPoints( WM->getBallPos(),posMyGoal);

      // determine where your front line intersects with the line from ball
            VecPosition posIntersect = lineFront.getIntersection( lineBall );

      // outside rectangle, use line at side to get intersection
            if (posIntersect.isRightOf( posRightTop ) )
            posIntersect = lineRight.getIntersection( lineBall );
            else if (posIntersect.isLeftOf( posLeftTop )  )
            posIntersect = lineLeft.getIntersection( lineBall );

            if( posIntersect.getX() < -49.0 )
            posIntersect.setX( -49.0 );

      // and move to this position
             if( posIntersect.getDistanceTo( WM->getAgentGlobalPosition() ) > 0.5 )
            {
            soc = moveToPos( posIntersect, PS->getPlayerWhenToTurnAngle() );
            ACT->putCommandInQueue( soc );
            ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
            }
            else
            {
            ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
            ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
            }
    }
    }
  }
  else if( WM->isFreeKickUs() == true || WM->isGoalKickUs() == true )
  {
    if( WM->isBallKickable() )
    {
      if( WM->getTimeSinceLastCatch() == 25 && WM->isFreeKickUs() )
      {
        // move to position with lesser opponents.
        if( WM->getNrInSetInCircle( OBJECT_SET_OPPONENTS,
                                          Circle(posRightTop, 15.0 )) <
            WM->getNrInSetInCircle( OBJECT_SET_OPPONENTS,
                                           Circle(posLeftTop,  15.0 )) )
          soc.makeCommand( CMD_MOVE,posRightTop.getX(),posRightTop.getY(),0.0);
        else
          soc.makeCommand( CMD_MOVE,posLeftTop.getX(), posLeftTop.getY(), 0.0);
        ACT->putCommandInQueue( soc );
      }
      else if( WM->getTimeSinceLastCatch() > 28 )
      {

        int temp=suitablePass1();
        soc = directPass(WM->getGlobalPosition((ObjectT)(temp)),PASS_FAST);
        ACT->putCommandInQueue( soc );

        //soc = kickTo( VecPosition(0,posAgent.getY()*2.0), 2.0 );
        //ACT->putCommandInQueue( soc );
      }
      else if( WM->getTimeSinceLastCatch() < 25 )
      {
        VecPosition posSide( 0.0, posAgent.getY() );
        if( fabs( (posSide - posAgent).getDirection() - angBody) > 10 )
        {
          soc = turnBodyToPoint( posSide );
          ACT->putCommandInQueue( soc );
        }
        ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
      }
    }
    else if( WM->isGoalKickUs()  )
    {
      ACT->putCommandInQueue( soc = intercept( true ) );
      ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
    }
    else
      ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
  }
  else
  {
     ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
     ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
  }
  return soc;
}

/*
 * Hauptschleife für Stürmer
 *
 * Die Stürmer agieren ausschließlich in der gegnerischen Spielfeldhälfte. Der Stürmer in der Mitte (Nummer 9) führt den Anstoß aus,
 * indem er zum Ball läuft und diesen nach hinten ans Mittelfeld passt (genauer: an Spieler Nummer 6).
 *
 * Befindet sich ein Stürmer im Ballbesitz, dribbelt er diesen solange Richtung Tor, bis er ihm entweder abgenommen wird oder
 * der Abstand zum Tor 20 (entspricht 1/5 der gesamten Spielfeldbreite) unterschreitet. In diesem Fall wird direkt aufs Tor geschossen.
 * Die restlichen Stürmer bewegen sich währenddessen parallel zum Ball mit.
 * Ist  der Gegner im Ballbesitz und befindet sich der Ball in der gegnerischen Hälfte, so versucht derjenige Stürmer, der am schnellsten
 * am Ball wäre, diesen abzunehmen.
 *
 * Autor: Tim A.
 *
 */
SoccerCommand Player::deMeer5_attacker()
{
    SoccerCommand attackerCommand;
    VecPosition   posAgent     = WM->getAgentGlobalPosition();
    VecPosition   posBall      = WM->getBallPos();
    double     dPosAgentX         = posAgent.getX();
    double     dPosAgentY         = posAgent.getY();
    double     dPosBallX        = posBall.getX();
    double     dPosBallY        = posBall.getY();
    int     iPlayerNr         = WM->getPlayerNumber();
    int        iTmp;


    // Vor Anstoß
    if(WM->isBeforeKickOff())
    {
        // Falls noch nicht auf Position, auf Position laufen
        if(!isPositionReached(WM->getStrategicPosition(), 2))
        {
            attackerCommand = teleportToPos(WM->getStrategicPosition());
            ACT->putCommandInQueue(attackerCommand);
            ACT->putCommandInQueue(turnNeckToObject(OBJECT_BALL, attackerCommand));
            ACT->putCommandInQueue(alignNeckWithBody());
        }

        // Wenn wir Anstoß haben und Player = 9
        if(WM->isKickOffUs() && WM->getPlayerNumber() == 9)
        {
            // Wenn Ball in Reichweite, annehmen und an Spieler 6 passen
            if(WM->isBallKickable())
            {
                attackerCommand = intercept(false);
                ACT->putCommandInQueue(attackerCommand);
                ACT->putCommandInQueue(turnNeckToObject(OBJECT_TEAMMATE_6, attackerCommand));
                ACT->putCommandInQueue(alignNeckWithBody());
                attackerCommand = pass(WM->getGlobalPosition(OBJECT_TEAMMATE_6));
                ACT->putCommandInQueue(attackerCommand);
            }
//            Wenn Ball nicht in Reichweite, Richtung Ball laufen
            else
            {
                attackerCommand = moveToPos(posBall, PS->getPlayerWhenToTurnAngle());
                ACT->putCommandInQueue(attackerCommand);
            }
        }
    }
    // Nach Anstoß -- "toter" Ball für uns -> Standardsituationen
    else if(WM->isDeadBallUs())
    {
        return standardsituationen();
    }
    // Nach Anstoß
    else
    {
        // Wir haben den Ball
        if(WM->isBallInOurPossesion())
        {
            // Falls Formation nicht offensiv --> auf offensiv stellen
            if(formations->getFormation() != FT_OFFENSIVE)
            {
                formations->setFormation(FT_OFFENSIVE);
                // Wer nicht am Ball ist, geht auf seine Position
                if(!WM->isBallKickable() && !isPositionReached(WM->getStrategicPosition(), 2))
                {
                    attackerCommand = moveToPos(WM->getStrategicPosition(), PS->getPlayerWhenToTurnAngle());
                    ACT->putCommandInQueue(attackerCommand);
                }
            }
            // Spieler am Ball
            if(WM->isBallKickable() && WM->getFastestInSetTo(OBJECT_SET_TEAMMATES, OBJECT_BALL, &iTmp) == WM->getAgentObjectType())
            {
                attackerCommand = intercept(false);
                ACT->putCommandInQueue(attackerCommand);
                // Wenn Abstand zum Tor < 20, aufs Tor schießen
                if(posAgent.getDistanceTo(*new VecPosition(50, 0)) < 20)
                {
                    attackerCommand = kickTo(*new VecPosition(50, 0), SS->getBallSpeedMax());
                    ACT->putCommandInQueue( attackerCommand );
                }
                // sonst: Richtung Tor dribbeln
                else
                {
                    attackerCommand = dribble(WM->getRelAngleOpponentGoal(), DRIBBLE_SLOW);
                    ACT->putCommandInQueue(attackerCommand);
                }
            }
            // Spieler nicht am Ball, aber am nächsten am Ball --> laufe Richtung Ball
            else if(dPosBallX > 0 && WM->getFastestInSetTo(OBJECT_SET_TEAMMATES, OBJECT_BALL, &iTmp) == WM->getAgentObjectType() && !isPositionReached(posBall, 0))
            {
                attackerCommand = moveToPos(posBall, PS->getPlayerWhenToTurnAngle());
                ACT->putCommandInQueue(attackerCommand);

            }
            // sonst --> laufe parallel zum Ball, max. bis -10
            else if(dPosBallX > -10 && dPosAgentX > -10&& fabs(dPosAgentX - dPosBallX) > 2.0)
            {
                attackerCommand = moveToPos(*new VecPosition(dPosBallX, WM->getStrategicPosition().getY()), PS->getPlayerWhenToTurnAngle());
                ACT->putCommandInQueue(attackerCommand);
            }
        }
        // Wir haben den Ball nicht
        else
        {
            // Falls Formation nicht defensiv --> auf defensiv stellen
            if(formations->getFormation() != FT_DEFENSIVE)
            {
                formations->setFormation(FT_DEFENSIVE);
                // Spieler hat keine Möglichkeit den Ball abzunehmen --> laufe zu Position
                if(!WM->isBallKickable())
                {
                    attackerCommand = moveToPos(WM->getStrategicPosition(), PS->getPlayerWhenToTurnAngle());
                    ACT->putCommandInQueue(attackerCommand);
                }
            }
            // Spieler kann Ball abnehmen --> Ball abnehmen
            if(WM->isBallKickable())
            {
                attackerCommand = intercept(false);
                ACT->putCommandInQueue(attackerCommand);
            }
            // Spieler kann Ball nicht abnehmen
            else
            {
                // Spieler ist am nächsten am Ball --> laufe Richtung Ball
                if(dPosBallX > 0 && WM->getFastestInSetTo(OBJECT_SET_TEAMMATES, OBJECT_BALL, &iTmp) == WM->getAgentObjectType())
                {
                    attackerCommand = moveToPos(WM->predictPosAfterNrCycles(OBJECT_BALL, 1), PS->getPlayerWhenToTurnAngle());
                    ACT->putCommandInQueue(attackerCommand);
                }
                // sonst --> laufe parallel zum Ball
                else if(dPosBallX >= 0 && dPosAgentX > -10 && fabs(dPosAgentX - dPosBallX) > 2.0)
                {
                    attackerCommand = moveToPos(*new VecPosition(dPosBallX, WM->getStrategicPosition().getY()), PS->getPlayerWhenToTurnAngle());
                    ACT->putCommandInQueue(attackerCommand);
                }
                // sonst --> laufe zur strategischen Position
                else if(!isPositionReached(WM->getStrategicPosition(), 2))
                {
                    attackerCommand = moveToPos(WM->getStrategicPosition(), PS->getPlayerWhenToTurnAngle());
                    ACT->putCommandInQueue(attackerCommand);
                }
            }
        }
    }

    return attackerCommand;
}

/*
 * In dieser Methode werden sämtliche Standardsituationen behandelt. Sie wird von allen Spielern aufgerufen, sobald eine der
 * folgenden Situationen eintritt und unser Team das Recht auf den Ball hat:
 *     - Einwurf
 *     - Freistoß
 *     - Eckball
 *     - Anstoß
 * (für Details siehe WorldModel::isDeadBallUs)
 *
 * Der Spieler, der am schnellsten am Ball ist, bewegt sich dorthin und passt sofort dem nächsten Spieler, der
 * mit Hilfe der Esuitable-Methode ermittelt wird. Der Rückgabewert stellt ein SoccerCommand-Objekt dar.
 *
 * Autor: Tim A.
 */
SoccerCommand Player::standardsituationen()
{
    int iTmp;
    SoccerCommand attackerCommand;
    VecPosition   posBall      = WM->getBallPos();

    if(WM->getFastestInSetTo(OBJECT_SET_TEAMMATES, OBJECT_BALL, &iTmp) == WM->getAgentObjectType())
    {
        if(WM->isBallKickable())
        {
            attackerCommand = intercept(false);
            ACT->putCommandInQueue(attackerCommand);
            attackerCommand = pass(WM->getGlobalPosition((ObjectT)(Esuitable())));
            ACT->putCommandInQueue(attackerCommand);
        }
        else
        {
            attackerCommand = moveToPos(posBall, PS->getPlayerWhenToTurnAngle());
            ACT->putCommandInQueue(attackerCommand);
        }
    }

    return attackerCommand;
}


/*
 * Passen des Balles zu einer Position target
 */
SoccerCommand Player::pass(VecPosition target)
{
    if (WM->getAgentGlobalPosition().getDistanceTo(target) > 20)
    {
        return directPass(target, PASS_FAST);
    }
    else
    {
        return directPass(target, PASS_NORMAL);
    }
}

/*
 * Jemand (tm) sollte hier noch was ergänzen
 */
bool Player::isPassingPractical(VecPosition source, VecPosition target)
{
    bool sinn = true;

    int anzGegner = WM->getNrInSetInCone(OBJECT_SET_OPPONENTS, 3, source, target);
    /*
    if (anzGegner > 1)
    {
        sinn = false;
    }
    else if (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS, Circle(target, 3)))
    {
        sinn = false;
    }
    else if(source.getDistanceTo(target) > 80)
    {
        sinn = false;
    }
    */

    sinn = anzGegner < 1
            && WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS, Circle(target, 3)) < 1
            && source.getDistanceTo(target) < 80;

    return sinn;
}

/*
 * Gibt an, ob der Spieler innerhalb von Radius <double entfernung = 2> um seine Position steht.
 * Falls ja wird in soc der Befehl den Ball anzuschauen gegeben, ansonsten die Bewegung zum
 * Punkt
 */
SoccerCommand Player::goToStrategicPosition(double entfernung)
{
    bool bIsNear = isPositionReached(WM->getStrategicPosition(), entfernung);
    //bool bIsNear = WM->getAgentGlobalPosition().getDistanceTo(WM->getBallPos()) < entfernung;
    // Verkürzt geschrieben in der Hoffnung mehr Performance zu bekommen
    /*if (bIsNear)
    {
        *soc = turnBodyToObject(OBJECT_BALL);
    }
    else
    {
        *soc = moveToPos(WM->getStrategicPosition(), PS->getPlayerWhenToTurnAngle());
    }*/
    return bIsNear ? turnBodyToObject(OBJECT_BALL) : moveToPos(WM->getStrategicPosition(), PS->getPlayerWhenToTurnAngle());
}

bool Player::isPositionReached(VecPosition target, double entfernung)
{
    return target.getDistanceTo(WM->getAgentGlobalPosition()) <= entfernung;
}

SoccerCommand Player::goToPosition(VecPosition target, double entfernung)
{
    if (WM->getAgentStamina().getStamina() < 2000)
    {
        entfernung += 3;
    }
    return WM->getAgentGlobalPosition().getDistanceTo(target) < entfernung ? turnBodyToObject(OBJECT_BALL) : moveToPos(target, PS->getPlayerWhenToTurnAngle());
}


/*
 * This is where the magic happens for all players between the attackers and
 * the defenders
 *
 * There are 3 Types of Midfielders:
 *     - Defensive
 *     - Normal
 *     - Offensive
 *
 * Defensive:
 *     The defensive midfielder is trying to annoy the opponent attacker in
 *     case of an attack. He's running towards the opponent and tries to take
 *     the ball.
 *     As soon as this player get's the ball he's trying to pass it to one of
 *     the normal midfielders. In case of having no normal midfielder to pass
 *     to, the defensive midfielder tries to pass the ball to the offensive
 *     midfielder.
 *
 * Normal:
 *  When the opponent is in attack mode, this player tries to get the ball
 *  if he is on the same side as the attacker. Otherwise he tries to be
 *  between the attacker and his companion on the other side.
 *  As soon as we get the ball, the normal midfielder tries to get the ball
 *  as near as possible towards the enemy goal. Therefore he is running
 *  straight to the front an passes the ball to the offensive midfielder
 *  or the midfielder on the other side as soon as an enemy is getting near
 *  him.
 *
 * Offensive
 *  This player just passes the ball to an attacker. When this player is
 *  not in control of the ball, he tries to break free from any opponent
 *  and to get in a line of sight for the player with the ball.
 *
 */
SoccerCommand Player::deMeer5_midfielder()
{
    SoccerCommand soc(CMD_ILLEGAL);

    VecPosition   posAgent = WM->getAgentGlobalPosition();
    VecPosition   posBall  = WM->getBallPos();
    int           iTmp;
    int           playerNumber = WM->getPlayerNumber();


    //------------------------------
    // Vor dem Start
    //------------------------------
    if (WM->isBeforeKickOff())
    {
        // In Formation laufen
        if (formations->getFormation() != FT_INITIAL || posAgent.getDistanceTo( WM->getStrategicPosition()) > 2.0)
        {
            formations->setFormation(FT_INITIAL);
            ACT->putCommandInQueue( soc = teleportToPos(WM->getStrategicPosition()));
        }
        else
        {
            ACT->putCommandInQueue(soc = turnBodyToPoint(posBall, 0));
            ACT->putCommandInQueue(alignNeckWithBody());
        }
        return soc;
    }

    //------------------------------
    // Standardsituationen
    //------------------------------
    if (WM->isDeadBallUs())
    {
        return standardsituationen();
    }


    //------------------------------
    // Ball suchen
    //------------------------------
    if( WM->getConfidence( OBJECT_BALL ) < PS->getBallConfThr() )
    {
        ACT->putCommandInQueue( soc = searchBall() );   // Wenn Ballposition nicht bekannt ist
        ACT->putCommandInQueue( alignNeckWithBody( ) );
    }


    //------------------------------
    // Normales Spiel
    //------------------------------

    formations->setFormation(FT_OFFENSIVE);

    if (playerNumber == 6)
    {
        //------------------------------
        // todo Spieler 6 - defensives Mittelfeld
        //------------------------------


        if (WM->isBallKickable())
        {
            // Ball liegt nah genug um gekickt zu werden
            // -> Spiele Ball an Mitspieler ab

            int iTarget = Esuitable();
            ObjectT target = (ObjectT) iTarget;


            if (WM->getConfidence(target) < 0.80 || iTarget == OBJECT_TEAMMATE_UNKNOWN)
            {
                // Wenn Position des Ziels nicht sicher: dribble
                //if (WM->getNrInSetInCircle(OBJECT_SET_OPPONENTS, Circle(target, 3)))
                if (posAgent.getX() < 5)
                {
                    DribbleT dribbleSpeed = WM->getAgentStamina().getStamina() < 3000 ? DRIBBLE_SLOW : DRIBBLE_FAST;
                    ACT->putCommandInQueue(soc = dribble(0, dribbleSpeed));
                }
                else
                {
                    // wenn zu weit vorne einfach abspielen
                    if (isPassingPractical(posAgent, WM->getGlobalPosition(OBJECT_TEAMMATE_11)))
                    {
                        pass(WM->getGlobalPosition(OBJECT_TEAMMATE_11));
                    }
                    else if (isPassingPractical(posAgent, WM->getGlobalPosition(OBJECT_TEAMMATE_10)))
                    {
                        pass(WM->getGlobalPosition(OBJECT_TEAMMATE_10));
                    }
                    else if (isPassingPractical(posAgent, WM->getGlobalPosition(OBJECT_TEAMMATE_9)))
                    {
                        pass(WM->getGlobalPosition(OBJECT_TEAMMATE_9));
                    }
                    else
                    {
                        double entfernung = posAgent.getDistanceTo(WM->getGlobalPosition(OBJECT_TEAMMATE_9));
                        target = OBJECT_TEAMMATE_9;

                        if (posAgent.getDistanceTo(WM->getGlobalPosition(OBJECT_TEAMMATE_10)) < entfernung)
                        {
                            entfernung = posAgent.getDistanceTo(WM->getGlobalPosition(OBJECT_TEAMMATE_10));
                            target = OBJECT_TEAMMATE_10;
                        }
                        if (posAgent.getDistanceTo(WM->getGlobalPosition(OBJECT_TEAMMATE_11)) < entfernung)
                        {
                            target = OBJECT_TEAMMATE_11;
                        }

                        ACT->putCommandInQueue(soc = pass(WM->getGlobalPosition(target)));
                    }
                }
            }
            else
            {
                // Wenn die Position des Ziels ziemlich sicher ist: abspielen
                ACT->putCommandInQueue(soc = pass(WM->getGlobalPosition(target)));
            }
        }
        // Ball nicht direkt kickbar
        else if (WM->getFastestInSetTo(OBJECT_SET_TEAMMATES, OBJECT_BALL, &iTmp) == WM->getAgentObjectType() && !WM->isDeadBallThem() )
        {
            // Spieler ist schnellster aus dem Team am Ball
            if (WM->getFastestInSetTo(OBJECT_SET_PLAYERS, OBJECT_BALL, &iTmp) == WM->getAgentObjectType())
            {
                // Spieler ist allgemein schnellster am Ball
                ACT->putCommandInQueue(soc=moveToPos(posBall, PS->getPlayerWhenToTurnAngle()));
            }
            else
            {
                // Gegner ist schneller am Ball
                soc = intercept(false); // intercept the ball
                ACT->putCommandInQueue(soc);
                ACT->putCommandInQueue(turnNeckToObject(OBJECT_BALL, soc));
            }
        }
        //Ball nicht kickbar und Spieler ist nicht schnellster am Ball
        else if (posBall.getX() > 0)
        {
            // Ball vor der Mittellinie -> auf Linie zwischen Ball und Mittellinie

            VecPosition posGoal = *(new VecPosition(-50, 0));
            Line lineBallGoal = Line::makeLineFromTwoPoints(posBall, posGoal);
            VecPosition zielAufLinie = lineBallGoal.getPointOnLineClosestTo(posAgent);
            bool bZielAufLinieZwischenBallUndTor = zielAufLinie.isBetweenX(posBall, posGoal) && zielAufLinie.isBetweenY(posBall, posGoal);
            if (isPositionReached(zielAufLinie) || posAgent.getDistanceTo(posBall) < posAgent.getDistanceTo(zielAufLinie) || !bZielAufLinieZwischenBallUndTor)
            {
                VecPosition ziel = VecPosition(0, posBall.getY());
                ACT->putCommandInQueue(soc = goToPosition(ziel));
            }
            else
            {
                VecPosition ziel = VecPosition(zielAufLinie.getX() > 0 ? 0 : zielAufLinie.getX(), zielAufLinie.getY());
                ACT->putCommandInQueue(soc = goToPosition(ziel));
            }

        }
        else
        {
            // Ball hinter der Mittellinie
            formations->setFormation(FT_DEFENSIVE);
            if(posBall.getX() < -30 || WM->isBallInOurPossesion())
            {
                // gehe auf Position
                soc = goToStrategicPosition(5);
                ACT->putCommandInQueue(soc);
            }
            else
            {
                // Wirkbereich
                Line lineBallGoal = Line::makeLineFromTwoPoints(posBall, VecPosition(-50, 0));
                VecPosition zielAufLinie = lineBallGoal.getPointOnLineClosestTo(posAgent);

                VecPosition target = posAgent.getDistanceTo(posBall) < posAgent.getDistanceTo(zielAufLinie) ? posBall : zielAufLinie;

                ACT->putCommandInQueue(soc = goToPosition(target));

            }

        }

    }
    else if (playerNumber == 7)
    {
        //------------------------------
        // todo Spieler 7 - Mittelfeld rechts (unten)
        //------------------------------

        if (WM->isBallKickable())
        {
            // Ball ist nahe genug um zuzutreten
            // -> wenn hinter dem Sturm dribblen, ansonsten abspielen

            // Interessante Verteidiger sind 9 und 10
            if (posAgent.getX() > WM->getGlobalPosition(OBJECT_TEAMMATE_9).getX()-5 || isPassingPractical(posAgent, WM->getGlobalPosition(OBJECT_TEAMMATE_9)))
            {
                // vor Spieler 9 (zuerst auf Spieler 9 prüfen, um den Torwart eher zu "verwirren"
                ACT->putCommandInQueue(soc = pass(WM->getGlobalPosition(OBJECT_TEAMMATE_9)));
            }
            else if (posAgent.getX() > WM->getGlobalPosition(OBJECT_TEAMMATE_10).getX()-5 || isPassingPractical(posAgent, WM->getGlobalPosition(OBJECT_TEAMMATE_10)))
            {
                // vor Spieler 10
                ACT->putCommandInQueue(soc = pass(WM->getGlobalPosition(OBJECT_TEAMMATE_10)));
            }
            else
            {
                //hinter den Angreifern

                DribbleT dribbleSpeed = WM->getAgentStamina().getStamina() < 3000 ? DRIBBLE_SLOW : DRIBBLE_FAST;
                ACT->putCommandInQueue(soc = dribble(0, dribbleSpeed));

            }
        }
        // Ball ist nicht direkt kickbar
        else if (WM->getFastestInSetTo(OBJECT_SET_TEAMMATES, OBJECT_BALL, &iTmp) == WM->getAgentObjectType() && !WM->isDeadBallThem() )
        {
            // Spieler ist der schnellste des eigenen Teams am Ball
            if (WM->getFastestInSetTo(OBJECT_SET_PLAYERS, OBJECT_BALL, &iTmp) == WM->getAgentObjectType())
            {
                // Spieler ist der schnellste Spieler des Feldes am Ball
                ACT->putCommandInQueue(soc=moveToPos(posBall, PS->getPlayerWhenToTurnAngle()));
            }
            else
            {
                // gegnerischer Spieler ist der Schnellste am Ball ist: Einschreiten und Ball stoppen
                soc = intercept(false); // intercept the ball
                ACT->putCommandInQueue(soc);
                ACT->putCommandInQueue(turnNeckToObject(OBJECT_BALL, soc));
            }
        }
        //Ball ist nicht direkt kickbar und Spieler nicht schnellster am Ball ist
        else if(posBall.getX() < -10)
        {
            // Defensive

            formations->setFormation(FT_DEFENSIVE);

            ACT->putCommandInQueue(soc = goToStrategicPosition());

        }
        else if (posBall.getX() > 25)
        {
            // Vor Offensive

            if (WM->isBallInOurPossesion())
            {
                // Wir haben den Ball
                ObjectT fastest = WM->getFastestInSetTo(OBJECT_SET_TEAMMATES_NO_GOALIE, OBJECT_BALL, &iTmp);
                if (fastest != WM->getAgentObjectType())
                {
                    // Spieler ist nicht schnellster am Ball
                    ACT->putCommandInQueue(soc = goToStrategicPosition());
                }
                else
                {
                    // Spieler ist schnellster am Ball
                    ACT->putCommandInQueue(soc = moveToPos(posBall, PS->getPlayerWhenToTurnAngle()));
                }
            }
            else
            {
                // Gegner hat den Ball

                if (posBall.getY() > 0)
                {
                    // Ball ist in der oberen Hälfte (nicht Bereich von Spieler 7)
                    ACT->putCommandInQueue(soc = goToStrategicPosition());
                }
                else
                {
                    // Ball ist in der eigenen Hälfte
                    // -> Spieler bewegt sich auf die Linie zwischen Ball und Tor
                    Line lineBallGoal = Line::makeLineFromTwoPoints(posBall, VecPosition(-50, 0));
                    VecPosition zielAufLinie = lineBallGoal.getPointOnLineClosestTo(posAgent);

                    VecPosition target = posAgent.getDistanceTo(posBall) < posAgent.getDistanceTo(zielAufLinie) ? posBall : zielAufLinie;

                    ACT->putCommandInQueue(goToPosition(target));
                }
            }
        }
        else
        {
            // Wirkbereich

            if (posBall.getY() > 0)
            {
                // nicht in eigenem Bereich
                if (WM->isBallInOurPossesion())
                {
                    ACT->putCommandInQueue(soc = goToStrategicPosition());
                }
                else
                {
                    ACT->putCommandInQueue(soc = goToPosition(VecPosition(posBall.getX(), posAgent.getY())));
                }
            }
            else
            {
                // Ball unter y = 0

                if (WM->isBallInOurPossesion())
                {
                    if (posBall.getX() > posAgent.getX())
                    {
                        // Spieler hinter dem Ball
                        ACT->putCommandInQueue(soc = goToStrategicPosition());
                    }
                    else
                    {
                        // Spieler vor dem Ball
                        ACT->putCommandInQueue(soc = turnBodyToObject(OBJECT_BALL));
                        ACT->putCommandInQueue(alignNeckWithBody());
                    }
                }
                else
                {
                    // Gegner hat den Ball
                    ACT->putCommandInQueue(soc = goToPosition(posBall));
                }
            }


        }

    }
    else if (playerNumber == 8)
    {
        //------------------------------
        // todo Spieler 8 - Mittelfeld links (oben)
        //------------------------------
        if (WM->isBallKickable())
        {
            // Ball ist nahe genug um zuzutreten
            // -> wenn hinter dem Sturm dribblen, ansonsten abspielen

            // Interessante Angreifer sind 9 und 11
            if (posAgent.getX() > WM->getGlobalPosition(OBJECT_TEAMMATE_9).getX()-5 || isPassingPractical(posAgent, WM->getGlobalPosition(OBJECT_TEAMMATE_9)))
            {
                // vor Spieler 9 (zuerst auf Spieler 9 prüfen, um den Torwart eher zu "verwirren"
                ACT->putCommandInQueue(soc = pass(WM->getGlobalPosition(OBJECT_TEAMMATE_9)));
            }
            else if (posAgent.getX() > WM->getGlobalPosition(OBJECT_TEAMMATE_11).getX()-5 || isPassingPractical(posAgent, WM->getGlobalPosition(OBJECT_TEAMMATE_9)))
            {
                // vor Spieler 11
                ACT->putCommandInQueue(soc = pass(WM->getGlobalPosition(OBJECT_TEAMMATE_11)));
            }
            else
            {
                //hinter den Angreifern

                DribbleT dribbleSpeed = WM->getAgentStamina().getStamina() < 3000 ? DRIBBLE_SLOW : DRIBBLE_FAST;
                ACT->putCommandInQueue(soc = dribble(0, dribbleSpeed));

            }
        }
        // Ball nicht direkt kickbar
        else if (WM->getFastestInSetTo(OBJECT_SET_TEAMMATES, OBJECT_BALL, &iTmp) == WM->getAgentObjectType() && !WM->isDeadBallThem() )
        {
            // Spieler ist schnellster aus dem Team am Ball
            if (WM->getFastestInSetTo(OBJECT_SET_PLAYERS, OBJECT_BALL, &iTmp) == WM->getAgentObjectType())
            {
                // Spieler ist insgesamt schnellster am Ball
                ACT->putCommandInQueue(soc=moveToPos(posBall, PS->getPlayerWhenToTurnAngle()));
            }
            else
            {
                // wenn der Spieler der schnellste am Ball ist: Einschreiten und Ball stoppen
                soc = intercept(false); // intercept the ball
                ACT->putCommandInQueue(soc);
                ACT->putCommandInQueue(turnNeckToObject(OBJECT_BALL, soc));
            }
        }
        else if(posBall.getX() < -10)
        {
            // Defensive

            formations->setFormation(FT_DEFENSIVE);

            ACT->putCommandInQueue(soc = goToStrategicPosition());

        }
        else if (posBall.getX() > 25)
        {
            // Vor Offensive

            if (WM->isBallInOurPossesion())
            {
                ObjectT fastest = WM->getFastestInSetTo(OBJECT_SET_TEAMMATES_NO_GOALIE, OBJECT_BALL, &iTmp);
                if (fastest != WM->getAgentObjectType())
                {
                    ACT->putCommandInQueue(soc = goToStrategicPosition());
                }
                else
                {
                    ACT->putCommandInQueue(soc = moveToPos(posBall, PS->getPlayerWhenToTurnAngle()));
                }
            }
            else
            {
                // Gegner hat den Ball

                if (posBall.getY() < 0)
                {
                    ACT->putCommandInQueue(soc = goToStrategicPosition());
                }
                else
                {
                    Line lineBallGoal = Line::makeLineFromTwoPoints(posBall, VecPosition(-50, 0));
                    VecPosition zielAufLinie = lineBallGoal.getPointOnLineClosestTo(posAgent);

                    VecPosition target = posAgent.getDistanceTo(posBall) < posAgent.getDistanceTo(zielAufLinie) ? posBall : zielAufLinie;

                    ACT->putCommandInQueue(goToPosition(target));
                }
            }
        }
        // posBall.getX() < 25 && > -10
        else
        {
            // Wirkbereich

            if (posBall.getY() < 0)
            {
                if (WM->isBallInOurPossesion())
                {
                    ACT->putCommandInQueue(soc = goToStrategicPosition());
                }
                else
                {
                    ACT->putCommandInQueue(soc = goToPosition(VecPosition(posBall.getX(), posAgent.getY())));
                }
            }
            else
            {
                // Ball über y = 0

                if (WM->isBallInOurPossesion())
                {
                    if (posBall.getX() > posAgent.getX())
                    {
                        // Spieler hinter dem Ball
                        ACT->putCommandInQueue(soc = goToStrategicPosition());
                    }
                    else
                    {
                        // Spieler vor dem Ball
                        ACT->putCommandInQueue(soc = turnBodyToObject(OBJECT_BALL));
                        ACT->putCommandInQueue(alignNeckWithBody());
                    }
                }
                else
                {
                    // Gegner hat den Ball
                    ACT->putCommandInQueue(soc = goToPosition(posBall));
                }
            }


        }

    }
    else
    {
        // falsch eingeteilter Spieler läuft an den oberen Spielfeldrand
        ACT->putCommandInQueue(soc = moveToPos(VecPosition((playerNumber-1) * 10 - 50, 50), PS->getPlayerWhenToTurnAngle()));
    }


    return soc;
}


SoccerCommand Player::deMeer5_defender( )
{
    SoccerCommand defenderCommand(CMD_ILLEGAL);

    VecPosition posAgent   = WM->getAgentGlobalPosition();
    VecPosition posBall    = WM->getBallPos();
    VecPosition posGoalie  = WM->getGlobalPosition(OBJECT_TEAMMATE_1);
    double      dPosAgentX = posAgent.getX();
    double      dPosAgentY = posAgent.getY();
    double      dPosBallX  = posBall.getX();
    double      dPosBallY  = posBall.getY();
    int         iPlayerNr  = WM->getPlayerNumber();
    int         iTmp;


    // Vor Anstoß
    if(WM->isBeforeKickOff())
    {
        //vor dem abstoß in die passende Position bewegen
        formations->setFormation(FT_INITIAL);
        if( posAgent.getDistanceTo( WM->getStrategicPosition() ) > 2.0 )
        {
            // Kick Off Formation einstellen
            defenderCommand = teleportToPos(WM->getStrategicPosition());
            ACT->putCommandInQueue(defenderCommand);
        }
        else                                            // nur Richtung Mitte drehen
        {
            ACT->putCommandInQueue( defenderCommand=turnBodyToPoint( VecPosition( 0, 0 ), 0 ) );
            ACT->putCommandInQueue( alignNeckWithBody( ) );
        }

        return defenderCommand;
    }
    else if(WM->isDeadBallUs())
    {
        return standardsituationen();
    }
    // Nach Anstoß
    else
    {
        // Wir im Ballbesitz
        if(WM->isBallInOurPossesion())
        {
            formations->setFormation(FT_OFFENSIVE);

            //bei eigenem Ballbesitz und Ball kickbar
            if(WM->isBallKickable())
            {
                //Suche nächsten möglichen Spieler und passe
                int x=Esuitable();
                if(x!=OBJECT_TEAMMATE_UNKNOWN)
                {
                    defenderCommand = pass(WM->getGlobalPosition((ObjectT)(x)));
                    ACT->putCommandInQueue(defenderCommand);
                }
                else //sonst laufe nach vorne und dribble den Ball
                {
                    ACT->putCommandInQueue(turnBodyToPoint(VecPosition (0, dPosAgentY), 0));
                    defenderCommand = dribble(0, DRIBBLE_WITHBALL);
                    ACT->putCommandInQueue(defenderCommand);
                }
            }
            //am nächsten zum Ball
            else if(dPosBallX < -20 && WM->getFastestInSetTo(OBJECT_SET_TEAMMATES, OBJECT_BALL, &iTmp) == WM->getAgentObjectType())
            {
                    //ball ab/annehmen
                    defenderCommand = intercept(false);
                    ACT->putCommandInQueue(defenderCommand);
            }
            //sonst in die normale Position bewegen
            else if( posAgent.getDistanceTo( WM->getStrategicPosition()) > 10.0 )
            {
                //in Position bewegen
                defenderCommand = moveToPos(WM->getStrategicPosition(), PS->getPlayerWhenToTurnAngle());
                ACT->putCommandInQueue(defenderCommand);
            }
            else                                            // nur Richtung Mitte drehen
            {
                //kopf in richtung Ball bewegen
                ACT->putCommandInQueue( defenderCommand=turnBodyToPoint( posBall, 0 ) );
            }
            return defenderCommand;
        }
        // Gegner im Ballbesitz
        else
        {
            //Formation auf Defensive setzen
            formations->setFormation(FT_DEFENSIVE);

            //Ball in eigener Hälfte (interessant)
            if( dPosBallX < 0)
            {
                //im Wirkbereich
                if( dPosBallY < dPosAgentY+20 && dPosBallY > dPosAgentY-20 )
                {
                    //Abstand zum Ball berechnen
                    if(posAgent.getDistanceTo(WM->predictPosAfterNrCycles(OBJECT_BALL, 0)) < 20.0)
                    {
                        //falls kleiner 20, zum Ball laufen
                        defenderCommand = intercept(false);
                        ACT->putCommandInQueue(defenderCommand);
                    }
                    else //sonst Ballposition in 4 Zyklen ermitteln und dorthin laufen
                    {
                        defenderCommand = moveToPos(WM->predictPosAfterNrCycles(OBJECT_BALL, 4), PS->getPlayerWhenToTurnAngle());
                        ACT->putCommandInQueue(defenderCommand);
                    }
                }
                else
                {
                    //körper in Richtung Ball drehen
                    ACT->putCommandInQueue( defenderCommand=turnBodyToPoint(posBall, 0));
                }
            }
            else
            {
                if( posAgent.getDistanceTo( WM->getStrategicPosition() ) > 10.0)
                {
                    //in Position bewegen
                    defenderCommand = moveToPos(WM->getStrategicPosition(), PS->getPlayerWhenToTurnAngle());
                    ACT->putCommandInQueue(defenderCommand);
                }
                else                                            // nur Richtung Mitte drehen
                {
                    //kopf in richtung Ball bewegen
                    ACT->putCommandInQueue( defenderCommand=turnBodyToPoint(posBall, 0));
                }
            }
        }
    }

    return defenderCommand;
}

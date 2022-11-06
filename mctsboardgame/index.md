# MctsBoardGame


## Introduction {#introduction}

This is a Unity implementation of a board game with some kinds of automatic players based on Monte Carlo Tree Search.


### Demo Link: {#demo-link}

<https://roxzmm.github.io/Mctsboardgame/>


## Attention {#attention}


### Settings {#settings}

-   The number of players could only be 2, or 3, or 4.
-   The number of characters for each player could only be 2, or 3, or 4.
-   The size of board should be large enough to hold all players' characters.


### MCTS {#mcts}

-   MctsDDA: This computer will always try to keep close with your scores and it will not try to beat you.
-   MctsVictory: This computer will try its best to beat you and it can beat random player simply indeed. But don't worry, it's still hard for this computer to beat a real human player now because I haven't done enough optimizations for its algorithm efficiency. So, I believe you can beat it hard.

Both MCTS computer players will take about 4 seconds during their playing turns. Please don't worry if the screen seems stucked, it happens just because the computer needs time to think how to beat you. And, of course, you have nearly infinite time during your turn, computers have no way to disturb you.


/* Uses the Q-value table to choose an action according to the Bolzman */
/* distribution with decreasing temperature                            */
/* Return value: number of the selected action                         */

#define STATES  82                          /* Number of states  */
#define ACTIONS 2                           /* Number of actions */

int bolzman_exploration(double q_values[ACTIONS][STATES]);

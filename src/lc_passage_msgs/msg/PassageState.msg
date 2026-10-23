std_msgs/Header header
string plan_id
string passage_id
string phase
float64 progress
bool locked
bool folded
bool stop
bool fault
# Intentional stops only; obstacle stops are still checked for normal progress.
bool intentional_wait

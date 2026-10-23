# A candidate becomes executable only when Controller.setPlan receives this path.
std_msgs/Header header
string plan_id
nav_msgs/Path path
float64[] arc_lengths
lc_passage_msgs/PassageTraversal[] traversals

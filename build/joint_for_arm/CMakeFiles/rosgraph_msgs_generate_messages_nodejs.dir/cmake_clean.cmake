file(REMOVE_RECURSE
  "joint_for_arm_node_automoc.cpp"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/rosgraph_msgs_generate_messages_nodejs.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()

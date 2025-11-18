# #{ get_elem_by_tag_name(sensor, tag_name
def get_elem_by_tag_name(sensor, tag_name):
    elem = sensor.getElementsByTagName(tag_name)
    if not elem:
        raise AssertionError(f"Missing <{tag_name}> tag.")

    if not elem[0].firstChild or not elem[0].firstChild.data:
        raise AssertionError(f"Empty <{tag_name}> tag value.")

    return elem[0].firstChild.data


# #}


# #{ get_topic_by_tag_name(sensor, tag_name)
def get_topic_by_tag_name(sensor, tag_name):
    elem = sensor.getElementsByTagName(tag_name)
    if not elem:
        raise AssertionError(f"Missing <{tag_name}> tag.")

    if not elem[0].firstChild or not elem[0].firstChild.data:
        raise AssertionError(f"Empty <{tag_name}> tag value.")

    elem = '/' + elem[0].firstChild.data
    return elem


# #}


# #{ str_to_pose(pose_str: str)
def check_str_to_pose(pose_str: str):
    parts = pose_str.split()
    if len(parts) != 6:
        return False
    return True


# #}


# #{ replace_last_topic_segment(topic: str, new_last: str) -> str
def replace_last_topic_segment(topic: str, new_last: str) -> str:
    parts = topic.split('/')
    parts[-1] = new_last
    return '/'.join(parts)


# #}

from __future__ import print_function, absolute_import
import rospy
import genpy
from std_srvs.srv import Empty
import yaml
from bson import json_util, Binary
import json

from pymongo.errors import ConnectionFailure

import importlib
from datetime import datetime


def msg_to_document(msg):
    d = {}

    slot_types = []
    if hasattr(msg,'_slot_types'):
        slot_types = msg._slot_types
    else:
        slot_types = [None] * len(msg.__slots__)


    for (attr, type) in zip(msg.__slots__, slot_types):
        d[attr] = sanitize_value(attr, getattr(msg, attr), type)

    return d

def sanitize_value(attr, v, type):

    if isinstance(v, str):
        if type == 'uint8[]':
            v = Binary(v)
        else:
            # ensure unicode
            try:
                v = str(v)
            except UnicodeDecodeError as e:
                # at this point we can deal with the encoding, so treat it as binary
                v = Binary(v)
        # no need to carry on with the other type checks below
        return v
    if isinstance(v, rospy.Message):
        return msg_to_document(v)
    elif isinstance(v, genpy.rostime.Time):
        return msg_to_document(v)
    elif isinstance(v, genpy.rostime.Duration):
         return msg_to_document(v)
    elif isinstance(v, list):
        result = []
        for t in v:
            if hasattr(t, '_type'):
                result.append(sanitize_value(None, t, t._type))
            else:
                result.append(sanitize_value(None, t, None))
        return result
    else:
        return v



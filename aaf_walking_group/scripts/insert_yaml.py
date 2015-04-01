#!/usr/bin/env python

import std_msgs.msg
from mongodb_store.message_store import MessageStoreProxy
import yaml
import json
import pprint
import argparse


def loadDialogue(inputfile, dataset_name):
    print "openning %s" % inputfile
    with open(inputfile) as f:
            content = f.readlines()
    print "Done"
    return content


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset_name", help="The name of the dataset. Saved in meta information using 'meta_name'", type=str)
    parser.add_argument("-i", "--input", help="Input yaml file", type=str, required=True)
    parser.add_argument("--collection_name", help="The collection name. Default: aaf_walking_group", type=str, default="aaf_walking_group")
    parser.add_argument("--meta_name", help="The name of the meta filed to store 'dataset_name' in. Default: waypoint_set", type=str, default="waypoint_set")
    args = parser.parse_args()

    msg_store = MessageStoreProxy(collection=args.collection_name)
    data = yaml.load(open(args.input))
    meta = {}
    meta[args.meta_name] = args.dataset_name
    pprint.pprint(meta)
    pprint.pprint(data)
    msg_store.insert(std_msgs.msg.String(json.dumps(data)), meta)

#!/usr/bin/env python

# Utility script to control process on the simulator container
#  written by Yosuke Matsusaka <yosuke.matsusaka@gmail.com>
#  distributed under apache2 license

import argparse
import argcomplete
import xmlrpclib
import pprint

parser = argparse.ArgumentParser()
parser.add_argument('command', choices=['launch', 'reset', 'status'], help='run roslaunch or reset the simulator container')
parser.add_argument('args', nargs='*', help='command arguments')
parser.add_argument('-n', '--name', default='simulator', help='specify name of the process (default: simulator)')
parser.add_argument('-e', '--env', default='', help='specify environment variables')

argcomplete.autocomplete(parser)

args = parser.parse_args()

s = xmlrpclib.ServerProxy('http://simulator:9001')

if args.command == 'launch':
    try:
        s.supervisor.stopProcess('simulator:{0}'.format(args.name))
    except xmlrpclib.Fault:
        pass
    try:
        s.twiddler.removeProcessFromGroup('simulator', args.name)
    except xmlrpclib.Fault:
        pass
    command = ['/ros_entrypoint.sh roslaunch']
    command.extend(args.args)
    s.twiddler.addProgramToGroup('simulator', args.name,
        {
            'command': ' '.join(command),
            'environment': args.env,
            'autostart': 'true',
            'autorestart': 'true',
            'stopwaitsecs': '30',
            'stdout_logfile': '/dev/stdout',
            'stdout_logfile_maxbytes': '0',
            'stderr_logfile': '/dev/stderr',
            'stderr_logfile_maxbytes': '0'
        })
elif args.command == 'reset':
    s.supervisor.stopProcess('simulator:{0}'.format(args.name))
    s.supervisor.startProcess('simulator:{0}'.format(args.name))
elif args.command == 'status':
    pprint.pprint(s.supervisor.getAllProcessInfo())

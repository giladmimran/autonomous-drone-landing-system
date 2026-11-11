"""Single-camera shared-array IR application. Flight commands are not implemented."""
import argparse
import json
import logging
from config import Protocol
from ir.morse_decoder import MorseDecoder
from ir.signal_validator import SignalValidator
from ir.simulation import simulate
from pixhawk.rtl_controller import dry_run

def run(observations, protocol, monitor=None):
    decoder, validator = MorseDecoder(protocol), SignalValidator(protocol)
    for timestamp, state, quality in observations:
        logging.info(json.dumps({'event': 'observation', 'time': timestamp, 'state': state, 'quality': quality}))
        if monitor:
            monitor.poll()
        if state is None:
            validator.disconnect()
        result = decoder.feed(timestamp, state, quality)
        if result:
            accepted, reason = validator.accept(*result, timestamp)
            logging.info(json.dumps({'event': 'message', 'message': result[0], 'reason': reason}))
            print(result[0], reason)
            if accepted:
                decision = dry_run(monitor.status() if monitor else {})
                print(decision)
                logging.info(decision)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--simulate', action='store_true')
    parser.add_argument('--message', required=True)
    parser.add_argument('--unit', required=True, type=float, help='Measured dot duration in seconds')
    parser.add_argument('--camera-config', help='Measured camera/detector JSON settings')
    parser.add_argument('--log', default='ir_events.log')
    parser.add_argument('--mavlink-device')
    parser.add_argument('--system-id', type=int)
    parser.add_argument('--component-id', type=int)
    args = parser.parse_args()
    protocol = Protocol(args.message, args.unit)
    logging.basicConfig(filename=args.log, level=logging.INFO, format='%(asctime)s %(message)s')
    monitor = None
    if args.mavlink_device:
        if args.simulate or args.system_id is None or args.component_id is None:
            parser.error('Real monitoring requires camera mode and explicit vehicle IDs')
        from pixhawk.pixhawk_interface import PixhawkMonitor
        monitor = PixhawkMonitor(args.mavlink_device, args.system_id, args.component_id)
    try:
        if args.simulate:
            source = simulate(protocol.message, protocol.unit, protocol.confirmations)
        else:
            if not args.camera_config:
                parser.error('Camera mode requires measured --camera-config')
            from ir.ir_camera_receiver import receive
            with open(args.camera_config) as stream:
                source = receive(json.load(stream))
        run(source, protocol, monitor)
    finally:
        if monitor:
            monitor.close()

if __name__ == '__main__':
    main()

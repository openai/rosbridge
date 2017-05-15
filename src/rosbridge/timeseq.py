#
# A Python trace generator & importer.
# Doesn't necessarily support all the features of the C++/nodejs version
#
import ujson, os, time, random, math, gzip, collections, copy, logging, gzip, zmq, threading
from os import path

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

TIMESEQ_CHUNK = 10.0

def chunk_of(sample_ts):
    return math.floor(sample_ts / TIMESEQ_CHUNK) * TIMESEQ_CHUNK

def get_chunks(begin_ts, end_ts):
    ret = []
    ts = chunk_of(begin_ts)
    while ts < end_ts:
        ret.append(ts)
        ts += TIMESEQ_CHUNK
    return ret

def scan_trace(dir, channels, sample_cb):
    manifest_fn = path.join(dir, 'manifest.json')
    with open(manifest_fn, 'rb') as f:
        manifest = ujson.load(f)
    if 0: print manifest

    begin_ts = manifest['beginTs']
    end_ts = manifest['endTs']
    print 'range', begin_ts, end_ts

    timeseq_infos = manifest['timeseqInfos']
    cur_samples = {}
    # There's some thought to saving memory here. Only load a chunk (of all timeseqs) at a time
    for chunk in get_chunks(begin_ts, end_ts):
        all_chunk_data = {}
        all_chunk_indexes = {}
        for tsi in timeseq_infos:
            ts_name = tsi['name'];
            if ('*' not in channels) and (ts_name not in channels): continue
            chunk_fn = path.join(dir, 'chunk_%s_%d.json.gz' % (ts_name, chunk))
            try:
                with gzip.open(chunk_fn, 'rb') as f:
                    print 'Reading', chunk_fn, '...'
                    all_chunk_data[ts_name] = ujson.load(f)
                    all_chunk_indexes[ts_name] = 0
            except:
                print chunk_fn, 'not found'
                if ts_name in all_chunk_data:
                    del all_chunk_data[ts_name]
                    del all_chunk_indexes[ts_name]
        while True:
            # Find the next item to change (smallest timestamp)
            cur_ts = end_ts + 1
            for ts_name in all_chunk_data:
                if all_chunk_indexes[ts_name] < len(all_chunk_data[ts_name]['times']):
                    ts1 = all_chunk_data[ts_name]['times'][all_chunk_indexes[ts_name]]
                    cur_ts = min(cur_ts, ts1)
            if cur_ts > end_ts: # Didn't find anything
                break
            # Now update cur_samples with all samples with matching timestamps
            for ts_name in all_chunk_data:
                if all_chunk_indexes[ts_name] < len(all_chunk_data[ts_name]['times']):
                    ts1 = all_chunk_data[ts_name]['times'][all_chunk_indexes[ts_name]]
                    if ts1 == cur_ts:
                        cur_samples[ts_name] = all_chunk_data[ts_name]['samples'][all_chunk_indexes[ts_name]]
                        all_chunk_indexes[ts_name] += 1
            # We copy so that customers can keep a copy that works after we mutate cur_samples again
            # scan_trace_deltat does this
            sample_cb(cur_ts, copy.copy(cur_samples))


def scan_trace_deltat(dir, channels, deltat, sample_pair_cb):
    queue = collections.deque()
    def sample_cb(ts, samples):
        queue.append((ts, samples))
        while len(queue) > 1:
            deltat1 = queue[-1][0] - queue[0][0]
            if deltat1 < deltat: break
            old = queue.popleft()
            if deltat1 >= 0.95 * deltat:
                if 0: print deltat1, ts, old[0]
                sample_pair_cb(old[1], samples)
    scan_trace(dir, channels, sample_cb)



def mk_rand_suffix(n):
  return ''.join([random.choice('abcdefghjkmnpqrstuvwxyz') for i in range(n)])

def mk_new_trace_name(script):
  def fmt2(v):
    ret = '%d' % v
    while len(ret) < 2:
        ret = '0' + ret
    return ret

  tm = time.localtime()
  ftime = fmt2(tm.tm_year) + '-' + fmt2(tm.tm_mon) + '-' + fmt2(tm.tm_mday) + '_' + fmt2(tm.tm_hour) + '-' + fmt2(tm.tm_min) + '-' + fmt2(tm.tm_sec)
  rand_suffix = mk_rand_suffix(3)

  return script + '_' + ftime + '_' + rand_suffix

ur_dir =  path.dirname(path.dirname(path.abspath(__file__)))
traces_dir = path.join(ur_dir, 'traces')

class TimeseqWriter(object):
    """

    """
    def __init__(self, trace_name, trace_dir):
        self.trace_name = trace_name
        self.trace_dir = trace_dir
        self.files_by_channel = {}
        self.chunks_by_channel = {}
        self.times_by_channel = {}
        self.begin_ts = None
        self.end_ts = 0
        self.schemas = {}
        self.trace_notes = {}
        self.timeseq_infos = []
        self.camera_threads = []
        self.is_open = True
        logger.info('Writing trace to %s', self.trace_dir)

    @classmethod
    def create(cls, script):
        trace_name = mk_new_trace_name(script)
        trace_dir = path.join(traces_dir, trace_name)
        os.mkdir(trace_dir)
        return cls(trace_name, trace_dir)

    def get_url(self):
        # WRITEME: use public url here
        return 'http://roboscope.sci.openai.org/#scope_' + self.trace_name

    def add_channel(self, channel_name, type):
        self.timeseq_infos.append({
            '__type': 'TimeseqInfo',
            'name': channel_name,
            'type': type,
        })

    def add_schema(self, type, *members):
        self.schemas[type] = {
            'typename': type,
            'hasArrayNature': False,
            'members': [{
                'memberName': name,
                'typename': member_type,
            } for name, member_type in members]
        }

    def add(self, ts, channel_name, sample):
        try:
            if self.is_open:
                if self.begin_ts is None: self.begin_ts = ts
                if ts > self.end_ts: self.end_ts = ts

                chunk = chunk_of(ts)
                if self.chunks_by_channel.get(channel_name, -1) != chunk:
                    self.close_chunk(channel_name)
                    self.open_chunk(channel_name, chunk)
                f = self.files_by_channel[channel_name]
                if len(self.times_by_channel[channel_name]) > 0:
                    f.write(',')
                ujson.dump(sample, f)
                self.times_by_channel[channel_name].append(ts)
        except OverflowError:
            logger.info("Failed to serialize sample. Might it have numpy arrays with float32s? Please convert numpy arrays to float64")
            raise


    def open_chunk(self, channel_name, chunk):
        fn = path.join(self.trace_dir, 'chunk_%s_%d.json.gz' % (channel_name, int(chunk)))
        f = gzip.open(fn, 'wb')
        self.files_by_channel[channel_name] = f
        self.chunks_by_channel[channel_name] = chunk
        self.times_by_channel[channel_name] = []

        f.write('{"samples":[')

    def close_chunk(self, channel_name):
        f = self.files_by_channel.get(channel_name, None)
        if f is not None:
            f.write('],"times":')
            ujson.dump(self.times_by_channel[channel_name], f)
            f.write('}')
            f.close()

        self.files_by_channel[channel_name] = None
        self.chunks_by_channel[channel_name] = -1
        self.times_by_channel[channel_name] = []

    def close(self):
        if self.is_open:
            for k,v in self.files_by_channel.items():
                self.close_chunk(k)

            manifest = {
                '__type': 'TraceManifest',
                'beginTs': self.begin_ts,
                'endTs': self.end_ts,
                'traceNotes': self.trace_notes,
                'timeseqInfos': self.timeseq_infos,
                'schemas': self.schemas,
                'hasLores': False,
                'hasHires': True,
            }
            f = open(path.join(self.trace_dir, 'manifest.json'), 'wb')
            ujson.dump(manifest, f)
            f.close()
            logger.info('Wrote trace to %s', self.trace_dir)
            self.is_open = False

    def upload_s3(self):
        os.system('cd %s && node timeseq/uploadTraceS3.js %s' % (ur_dir, self.trace_dir))

    def stop_axis_video(self):
        self.want_axis_video = False

    def start_axis_video(self, timeseq_name, daemon_endpoint, resolution, ipaddr, auth_header, remote_traces_dir, local_link_prefix):
        self.want_axis_video = True

        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.DEALER)
        logger.info('Connecting to %s...', daemon_endpoint)
        self.sock.connect(daemon_endpoint)
        logger.info('Connected to %s', daemon_endpoint)

        self.add_schema('VideoFrame',
                        ('url', 'string'))
        self.add_channel(timeseq_name, 'VideoFrame')

        def thread_main():
            delta = None
            rpc_count = 0
            while True:
                want = self.want_axis_video
                req = {
                    '__type': 'AxisCameraDaemonReq',
                    'traceSpec': {
                        '__type': 'VideoTraceSpec',
                        'traceDir': path.join(remote_traces_dir, self.trace_name),
                        'traceName': self.trace_name,
                        'timeseqName': timeseq_name,
                    },
                    'cameraConfig': {
                        '__type': 'AxisCameraConfig',
                        'ipaddr': ipaddr,
                        'url': '/axis-cgi/mjpg/video.cgi?compression=30&rotation=0&resolution=' + resolution,
                        'authHeader': auth_header,
                    },
                    'txTime': time.time(),
                    'recordFor': 10.0 if want else 0.0,
                }
                rpc_id = 'rpc%d' % rpc_count
                rpc_count += 1
                tx = ['record', rpc_id, ujson.dumps(req, escape_forward_slashes=False)]
                if 1: logger.info('camera %s < %s', daemon_endpoint, tx)
                self.sock.send_multipart(tx, flags=0, copy=True, track=False)

                rx = self.sock.recv_multipart(flags=0, copy=True, track=False)
                rx_time = time.time()
                if 1: logger.info('%s > %s %s', daemon_endpoint, rx[1], rx[2])
                rpc_id2 = rx[0]
                rpc_err = ujson.loads(rx[1])
                rpc_result = ujson.loads(rx[2])
                if rpc_err:
                    logger.info('%s > error %s', daemon_endpoint, rpc_err)
                    break

                min_delta = rpc_result['txTime'] - rx_time
                max_delta = rpc_result['txTime'] - rpc_result['reqTxTime']
                if delta is None:
                    delta = (min_delta + max_delta) * 0.5
                delta = max(min_delta, min(max_delta, delta))
                if 0: logger.info("timing %0.6f %+0.6f %+0.6f min_delta=%+0.6f max_delta=%+0.6f delta=%+0.6f",
                    rpc_result['reqTxTime'], rpc_result['txTime'], rx_time, min_delta, max_delta, delta)

                rep_times = rpc_result['times']
                rep_samples = rpc_result['samples']
                for ts, sample in zip(rep_times, rep_samples):
                    self.add(ts - delta, timeseq_name, sample)
                rep_chunks = rpc_result['chunks']
                for chunk_ts in rep_chunks:
                    chunk_fn = 'chunk_%s_%.0f.video' % (timeseq_name, chunk_ts)
                    chunk_path = path.join(self.trace_dir, chunk_fn)
                    src_file = path.join(local_link_prefix, self.trace_name, chunk_fn)
                    logger.info('Create symlink(%s, %s)  llp=%s rtd=%s', src_file, chunk_path, local_link_prefix, remote_traces_dir)
                    os.symlink(src_file, chunk_path)

                if not want: break
                time.sleep(0.1)


        thr = threading.Thread(target = thread_main)
        thr.daemon = True
        thr.start()
        self.camera_threads.append(thr)

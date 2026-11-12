# Compact exports for completed ac jobs

Keep the compressed source bags, init records, automatic-job progress, acquisition
quality reports and local GelSight timing logs as the archival record. Export is
a disposable derived view, not a replacement. Do not delete corrupt bags: exports
report which completed attempts cannot be read, without silently treating them
as valid or initiating collection again. No automatic salvage is performed.

## Selection and layout

`extract_all` now requires --progress and --out; its default is preview-only.
The new ac route uses /trial_event, not the legacy /trial_marker parser. It selects
only each trial's unique final completed attempt from the supplied progress JSON,
then matches its START/END pair by trial index and attempt time range. Aborted,
unverified and earlier attempts never enter the learning view. Every exported
trial must contain both GelSight streams and both force streams. Missing
START/END, invalid JPEG type, unreadable bag or absent topic is reported as failed
or incomplete. A session containing a corrupt compressed segment is not exported
from its readable prefix. Other sessions can continue.

This command targets ac jobs. Legacy AP/batch use of extract_batch_bag remains a
separate legacy exporter; this change does not reinterpret those markers.

Each output session has:

- images/left and images/right: original JPEG payload bytes, unchanged. No PNG,
  video encoding, image decoding, resize, recompression or frame downsampling.
- image_index.csv: trial ID, topic, source Header ns, bag receive ns, split database
  index, message ID, original format, byte count and image path. Filenames include
  integer Header time and unique database/message IDs.
- force_left.csv and force_right.csv: full-rate six-axis measurements, both ns
  timestamps, trial ID, sample identity and per-trial baseline-relative Fy.
  Spikes and nonfinite values remain; nonfinite counts are reported separately.
- trial_events.json and manifest.json: event list, matched trial windows, frozen
  conditions, attempts/tare, missing-topic counts, image/CSV bytes, source metadata
  and exporter hashes. Export completeness does not certify physical sync.
- session_records/: copies of available local timing logs, quality, init reference,
  collection provenance and trial conditions/plan. Original init samples remain
  in the init_records path referenced by init_reference; retain that archive too.

Sensor membership is START <= sensor Header <= END. This differs deliberately
from legacy exports that cut by bag receive time; it avoids excluding sensor
samples simply because delivery was delayed. There is no contact inference or
automatic timestamp correction. All local timing logs are copied once per
session, including records outside trial windows, so delayed metadata is not
cut away. Other ROS topics, RealSense pixels, pre/post-trial force samples and
calibration data remain available in the original bag; they are not expanded
by default. Images are written once per selected message and indexed by trial,
not duplicated into multiple trial directories. Training must read image_index
and group by trial_id. Force and images are not pre-aligned or resampled.

## Preview, then explicit write

Build data_capture_tools, source install/setup.bash, then run:

```bash
ros2 run data_capture_tools extract_all \
  --progress /mnt/nvme1/tsumura/masters_thesis/data/object7/automatic_jobs/20261105_161811_4e274922.json \
  --out ./object7_export_preview_20261105
```

No JPEGs or force CSV are written in preview. It still reads/decompresses bags
to determine actual byte counts; it can take time. One compressed split is
expanded into a temporary database at a time and automatically cleaned up. The
event pass and sensor pass decompress separately, bounding temporary disk use
to one split rather than all parallel bags. Leave room for one expanded split.

Inspect report.json before writing. It contains per-session errors and the
affected trial IDs, expected/completed coverage, original JPEG bytes, calculated
CSV bytes and copied-sidecar bytes. Estimates exclude manifest/event JSON,
source_progress.json and filesystem allocation overhead. Source JPEGs are
already compressed, so many small JPEG files need more disk blocks than their
byte counts suggest. Large failed sessions are not included in the byte total;
a partial report must not be taken as the size of the entire object.

For explicit export use a different, new output directory on a drive with space:

```bash
ros2 run data_capture_tools extract_all \
  --progress /mnt/nvme1/tsumura/masters_thesis/data/object7/automatic_jobs/20261105_161811_4e274922.json \
  --out /mnt/nvme1/tsumura/masters_thesis/exports/object7_20261105 \
  --write
```

Both modes refuse existing output paths and never overwrite prior exports.
Preview creates only source_progress.json and report.json. --session NAME may
be repeated to select exact session basenames, e.g. before exporting an entire
object. Run offline, not alongside ongoing acquisition on the same workstation.
A nonzero exit code means one or more sessions failed/incomplete; read report.json.
This is not a new acquisition abort condition.

The same CLI is available via `/usr/bin/python3 -m data_capture_tools.compact_export`.
No additional extraction YAML is needed for this default GelSight/force view.
The original JPEG and acquisition coordinates are retained so future processing
can choose crops, masks, temporal windows and sample rates without recollection.


## 波形確認用の補助CSV

新しいextract_allでは、各エクスポート済みセッションへ次の3ファイルを追加する。

- `robot_action_event.csv` に、記録された指令・成功・保持の時刻、phase、目標幅・速度。
- `gripper_width.csv` に、ドライバが出力した幅mmとbag受信時刻。元メッセージにHeaderがないため、source_stamp_nsは空欄。
- `gripper_status.csv` に、Header時刻、raw入力レジスタ、読み取り時間。

補助CSVはSTART直前最大0.5秒からENDまで。JPEG・力覚CSV本体のSTART〜ENDという範囲と元の値は維持する。プレビュー時のCSV容量見積もりにも補助CSVを含める。

すでに展開した旧出力も `plot_object_acquisition.py --export-dir <出力先>` で利用できる。補助CSVがない場合は、対応するパネル・動作時刻の表示を省く。新しい補助CSVを得るための再エクスポートは、必要になったときだけ行う。

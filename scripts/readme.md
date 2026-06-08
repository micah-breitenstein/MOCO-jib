python3 camera_trigger_server.py

This runner now uses the canonical timelapse tools from ../holy-grail-timelapse.

Orin timelapse sync automation:

1) One-time sync + process for all remote timelapse_[timestamp] folders:
	./sync_orin_timelapse.sh

2) Continuous sync for long captures (resume-safe):
	./sync_orin_timelapse.sh --watch --interval 30

3) Choose how remote directories are shown in the listing section:
	./sync_orin_timelapse.sh --list-view all
	./sync_orin_timelapse.sh --list-view optimal

4) Sync only one timelapse folder (its remote optimal/ is mirrored locally):
	./sync_orin_timelapse.sh --folder timelapse_20260602_204016

5) Use a different remote source path if needed:
	./sync_orin_timelapse.sh \
	  --folder timelapse_20260602_204016 \
	  --remote-dir /home/micah/projects/holy-grail/holy-grail-timelapse/timelapse

Notes:
- The script discovers remote folders named timelapse_[timestamp].
- The script prints a markdown section header: ## Timelapse Directories.
- Use --list-view all or --list-view optimal to control what path suffix appears in that list.
- For each timelapse folder, only optimal/ is synced.
- Local mirror path is timelapse_orin/timelapse_[timestamp]/optimal/.
- New files are synced incrementally using rsync.
- Processing only runs on files not listed in .sync_state/processed_files.txt (per timelapse folder).
- Analyzer outputs are written to .sync_state/analysis/*.json.

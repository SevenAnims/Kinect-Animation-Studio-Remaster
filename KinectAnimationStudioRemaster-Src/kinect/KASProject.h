#pragma once

class KRetargeter;
class KBodyExporter;
class KBodyVisualizer;

/*
 Project save / load (.KAS).

 The file stores everything needed to reopen a session exactly as it was:
   - the imported character (source FBX path),
   - the import transform (size + rotation),
   - the bone -> Kinect joint mapping,
   - the full recorded take (mocap frames).

 Binary format, magic "KAS1" + version 1.
*/
class KASProject
{
public:
	/// <summary>
	/// Saves the current project to the given .KAS file. Returns true on success.
	/// </summary>
	static bool save(const char *path, const KRetargeter *retargeter, const KBodyExporter *exporter);
	static bool save(const char *path, const KRetargeter *retargeter, const KBodyExporter *exporter, const KBodyVisualizer *visualizer);

	/// <summary>
	/// Loads a .KAS project file, replacing the current character, mapping,
	/// transform and take. Returns true on success.
	/// </summary>
	static bool load(const char *path, KRetargeter *retargeter, KBodyExporter *exporter);
	static bool load(const char *path, KRetargeter *retargeter, KBodyExporter *exporter, KBodyVisualizer *visualizer);
};

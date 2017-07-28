echo ".BATCH Convert to MP4 from Pgm. 2017 kostasl"
echo "Make HD Vids from Image Sequence Dirs.."
for dir in `find $1 -type d`
do
  #test -d "$dir" || continue
	files=($dir/*.pgm)
	if [ ${#files[@]} -gt 0 ]; then
#	 echo $dir;
	 echo "Found PGM Image FILES..in $dir";
#	 Make Video
	  filename=${dir//[\/]/_}
	  filename=${filename//[.]/}
	  echo $filename
	   avconv -framerate 300 -i $dir/%10d.pgm -c:v libx264 -crf 17 -crf_max 45 $dir/../$filename.mp4
	fi
	
	

#	if [ -e "$(test -f $dir/*.cpp)" ];
#	then 
#		echo "Found Files";
#	else
#	  echo "empty (or does not exist)"
#	fi
#	

# Do something with $dir...
done


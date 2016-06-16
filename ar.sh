cd log/
for file in *.log; do
    mv ${file} $(date "+%s")_${file}
done
cd ..
mv log/* archive

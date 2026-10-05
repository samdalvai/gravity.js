// import { runModified, runOriginal } from './functions';
import { runOriginal } from './functions';
import { benchmark } from './registry';

benchmark('original', runOriginal);
// benchmark('modified', runModified);
